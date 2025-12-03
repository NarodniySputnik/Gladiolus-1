import numpy as np
from skyfield.api import load, EarthSatellite, utc
from datetime import datetime, timedelta
from scipy.spatial.transform import Rotation
from pygeomag import GeoMag
import math
import warnings
import time

warnings.filterwarnings('ignore')

class SatelliteEmulator:
    def __init__(self, tle_line1, tle_line2, initial_quat=None, start_time=None, use_current_time=True):
        """Инициализация эмулятора спутника CubeSat 3U с тепловой моделью."""
        self.ts = load.timescale()
        
        # Загрузка данных для преобразований
        print("Загрузка эфемерид...")
        self.eph = load('de421.bsp')  # Эфемериды
        self.sun = self.eph['sun']
        self.earth = self.eph['earth']
        
        # Создание объекта спутника
        self.sat_obj = EarthSatellite(tle_line1, tle_line2, 'CubeSat 3U', self.ts)
        
        # Инициализация модели магнитного поля
        print("Инициализация модели магнитного поля WMM...")
        self.geo_mag = GeoMag()
        
        # Определение начального времени
        if use_current_time:
            start_time = datetime.now().astimezone(tz=utc)
            print(f"Используется текущее время: {start_time}")
        elif start_time is None:
            start_time = datetime.now().astimezone(tz=utc)
        else:
            if start_time.tzinfo is None:
                start_time = start_time.replace(tzinfo=utc)
                
        self.start_time = start_time
        self.current_time = start_time
        
        # Начальное состояние
        self.pos_eci_km = np.array([0., 0., 0.])
        self.vel_eci_km_s = np.array([0., 0., 0.])
        
        # Геодезические координаты
        self.lat_deg = 0.0
        self.lon_deg = 0.0
        self.alt_km = 0.0
        
        # Ориентация
        if initial_quat is None:
            self.attitude_quat = np.array([0., 0., 0., 1.])
        else:
            self.attitude_quat = initial_quat / np.linalg.norm(initial_quat)
            
        # Медленное вращение
        self.angular_vel = np.array([
            math.radians(0.001),
            math.radians(0.003),
            math.radians(0.01)
        ])
        
        # Переменные для направления на Солнце и Землю
        self.sun_direction_eci = np.array([0., 0., 0.])
        self.sun_direction_body = np.array([0., 0., 0.])
        self.earth_direction_body = np.array([0., 0., 0.])
        self.sun_visible = False
        self.earth_visible = False
        
        print(f"Задано вращение: X={math.degrees(self.angular_vel[0]):.3f} °/с, "
              f"Y={math.degrees(self.angular_vel[1]):.3f} °/с, "
              f"Z={math.degrees(self.angular_vel[2]):.3f} °/с")
        
        self.magnetic_field_body = np.array([0., 0., 0.])
        self.solar_power = 0.0
        
        # ===== МОДЕЛЬ СОЛНЕЧНЫХ БАТАРЕЙ =====
        self.solar_constant = 1361.0
        self.albedo_constant = 1361.0 * 0.3
        self.panel_efficiency = 0.28
        
        # Определяем грани CubeSat 3U
        self.solar_panels = {
            '+X': {'normal': np.array([1., 0., 0.]), 'area': 0.0, 'enabled': False, 'has_panel': False},
            '-X': {'normal': np.array([-1., 0., 0.]), 'area': 0.0, 'enabled': False, 'has_panel': False},
            '+Y': {'normal': np.array([0., 1., 0.]), 'area': 0.03, 'enabled': True, 'has_panel': True},
            '-Y': {'normal': np.array([0., -1., 0.]), 'area': 0.03, 'enabled': True, 'has_panel': True},
            '+Z': {'normal': np.array([0., 0., 1.]), 'area': 0.01, 'enabled': True, 'has_panel': True},
            '-Z': {'normal': np.array([0., 0., -1.]), 'area': 0.01, 'enabled': True, 'has_panel': True}
        }
        
        self.total_panel_area = sum([panel['area'] for panel in self.solar_panels.values() 
                                     if panel['enabled']])
        
        # ===== ТЕПЛОВАЯ МОДЕЛЬ CUBESAT 3U =====
        self.material_properties = {
            'clear_anodized_al': {
                'solar_absorptivity': 0.37,
                'infrared_emissivity': 0.80,
                'thermal_conductivity': 237.0,
                'specific_heat': 900.0,
                'density': 2700.0,
                'thickness': 0.0015
            }
        }
        
        # Геометрические параметры
        self.cubesat_dimensions = {
            'length': 0.3,
            'width': 0.1,
            'height': 0.1,
            'volume': 0.3 * 0.1 * 0.1,
            'surface_area': 2 * (0.3*0.1 + 0.3*0.1 + 0.1*0.1)
        }
        
        # Масса и теплоёмкость
        self.mass = 4.0
        self.heat_capacity = self.mass * self.material_properties['clear_anodized_al']['specific_heat']
        
        # Начальные температуры
        self.internal_temperature = 293.15
        self.surface_temperatures = {
            '+X': 293.15, '-X': 293.15,
            '+Y': 293.15, '-Y': 293.15,
            '+Z': 293.15, '-Z': 293.15
        }
        
        # Внутренние источники тепла
        self.internal_heat_sources = {
            'electronics': 1.5,
            'battery': 0.5,
            'transceiver': 2.0,
            'total': 4.0
        }
        
        # Альбедо Земли и ИК излучение
        self.earth_albedo = 0.3
        self.earth_ir_flux = 240.0
        
        # Константа Стефана-Больцмана
        self.stefan_boltzmann = 5.670374419e-8
        
        print(f"\nТЕПЛОВАЯ МОДЕЛЬ CUBESAT 3U:")
        print(f"  Альбедо Земли: {self.earth_albedo*100}% от солнечной постоянной")
        
        # Кэш
        self._current_gmst = 0.0
        self._current_rotation_matrix_ecef_to_eci = np.eye(3)
        
        print("Эмулятор инициализирован успешно!")
    
    def _calculate_gmst(self, utc_time):
        """Вычисление гринвичского среднего звёздного времени."""
        if utc_time.tzinfo is None:
            utc_time = utc_time.replace(tzinfo=utc)
            
        jd = (utc_time - datetime(2000, 1, 1, 12, 0, 0, tzinfo=utc)).total_seconds() / 86400.0 + 2451545.0
        
        t = (jd - 2451545.0) / 36525.0
        gmst_seconds = 67310.54841 + (876600 * 3600 + 8640184.812866) * t + 0.093104 * t**2 - 0.0000062 * t**3
        gmst_seconds = gmst_seconds % 86400.0
        
        return (gmst_seconds / 43200.0) * math.pi
    
    def _get_rotation_matrix_ecef_to_eci(self, gmst_rad):
        """Матрица вращения из ECEF в ECI."""
        cos_gmst = math.cos(gmst_rad)
        sin_gmst = math.sin(gmst_rad)
        
        return np.array([
            [cos_gmst, -sin_gmst, 0],
            [sin_gmst,  cos_gmst, 0],
            [0,         0,        1]
        ])
    
    def _calculate_rotation_matrix_ned_to_ecef(self, lat_rad, lon_rad):
        """Матрица вращения из NED в ECEF."""
        sin_lat = math.sin(lat_rad)
        cos_lat = math.cos(lat_rad)
        sin_lon = math.sin(lon_rad)
        cos_lon = math.cos(lon_rad)
        
        return np.array([
            [-sin_lat * cos_lon, -sin_lon, -cos_lat * cos_lon],
            [-sin_lat * sin_lon,  cos_lon, -cos_lat * sin_lon],
            [cos_lat,            0,       -sin_lat]
        ])
    
    def _calculate_magnetic_field(self, lat_deg, lon_deg, alt_km, time_utc):
        """Рассчитывает геомагнитное поле в точке."""
        try:
            if time_utc.tzinfo is None:
                time_utc = time_utc.replace(tzinfo=utc)
            
            year_decimal = time_utc.year + (time_utc.timetuple().tm_yday - 1) / 365.25
            
            result = self.geo_mag.calculate(
                glat=lat_deg,
                glon=lon_deg,
                alt=alt_km,
                time=year_decimal
            )
            
            return np.array([result.x, result.y, result.z])
            
        except Exception as e:
            print(f"Ошибка при расчёте магнитного поля: {e}")
            return np.array([0.0, 0.0, 0.0])
    
    def _calculate_sun_and_earth_positions(self, t):
        """Рассчитывает положение Солнца и Земли относительно спутника."""
        try:
            # Положение Солнца в ECI
            sun_pos = self.sun.at(t)
            
            # Вектор от спутника к Солнцу в ECI
            vector_to_sun_eci = sun_pos.position.km - self.pos_eci_km
            distance_to_sun = np.linalg.norm(vector_to_sun_eci)
            
            # Вектор от спутника к Земле
            vector_to_earth_eci = -self.pos_eci_km
            
            if distance_to_sun > 0:
                # Единичный вектор направления на Солнце в ECI
                sun_dir_eci = vector_to_sun_eci / distance_to_sun
                
                # Преобразуем в связанную систему координат
                rot_body_to_eci = Rotation.from_quat(self.attitude_quat)
                R_eci_to_body = rot_body_to_eci.as_matrix().T
                sun_dir_body = R_eci_to_body @ sun_dir_eci
                
                # Проверяем видимость Земли
                distance_to_earth = np.linalg.norm(vector_to_earth_eci)
                if distance_to_earth > 0:
                    earth_dir_eci = vector_to_earth_eci / distance_to_earth
                    earth_dir_body = R_eci_to_body @ earth_dir_eci
                    earth_visible = True
                else:
                    earth_dir_body = np.array([0., 0., 0.])
                    earth_visible = False
                
                # Проверяем нахождение в тени Земли
                earth_radius_km = 6378.137
                sat_altitude = np.linalg.norm(self.pos_eci_km) - earth_radius_km
                
                if distance_to_earth > 0:
                    earth_direction = vector_to_earth_eci / distance_to_earth
                    dot_product = np.dot(sun_dir_eci, earth_direction)
                    
                    # Если Солнце "позади" Земли
                    if dot_product < 0 and sat_altitude < 1000:
                        earth_angular_radius = math.asin(earth_radius_km / np.linalg.norm(self.pos_eci_km))
                        sun_earth_angle = math.acos(min(1.0, max(-1.0, dot_product)))
                        
                        if sun_earth_angle < earth_angular_radius:
                            return sun_dir_body, sun_dir_eci, earth_dir_body, False, earth_visible
                
                return sun_dir_body, sun_dir_eci, earth_dir_body, True, earth_visible
            
            return (np.array([0., 0., 0.]), np.array([0., 0., 0.]), 
                    np.array([0., 0., 0.]), False, False)
            
        except Exception as e:
            print(f"Ошибка при расчёте положения Солнца и Земли: {e}")
            return (np.array([0., 0., 0.]), np.array([0., 0., 0.]), 
                    np.array([0., 0., 0.]), False, False)
    
    def _calculate_solar_power_with_albedo(self):
        """
        Расчёт мощности солнечных батарей с учётом:
        1. Прямого солнечного излучения
        2. Альбедо Земли
        """
        total_power = 0.0
        panel_powers = {}
        power_details = {}
        
        if not self.sun_visible and not self.earth_visible:
            return total_power, panel_powers, power_details
        
        # Матрица вращения
        rot_body_to_eci = Rotation.from_quat(self.attitude_quat)
        R_body_to_eci = rot_body_to_eci.as_matrix()
        
        for panel_name, panel in self.solar_panels.items():
            if not panel['enabled']:
                continue
            
            panel_power = 0.0
            direct_power = 0.0
            albedo_power = 0.0
            
            # Нормаль панели в связанной системе
            normal_body = panel['normal']
            
            # Преобразуем нормаль в ECI
            normal_eci = R_body_to_eci @ normal_body
            
            # 1. ПРЯМОЕ СОЛНЕЧНОЕ ИЗЛУЧЕНИЕ
            if self.sun_visible:
                cos_angle_sun = np.dot(normal_eci, self.sun_direction_eci)
                if cos_angle_sun > 0:
                    direct_power = (self.solar_constant * panel['area'] * 
                                  self.panel_efficiency * cos_angle_sun)
                    panel_power += direct_power
            
            # 2. АЛЬБЕДО ЗЕМЛИ
            if self.earth_visible:
                # Направление к центру Земли
                earth_direction = -self.pos_eci_km / np.linalg.norm(self.pos_eci_km)
                
                # Косинус угла между нормалью панели и направлением на Землю
                cos_angle_earth = np.dot(normal_eci, earth_direction)
                
                if cos_angle_earth > 0:  # Панель смотрит на Землю
                    # Высота спутника
                    earth_radius = 6378.137
                    altitude = np.linalg.norm(self.pos_eci_km) - earth_radius
                    
                    # Угловой радиус Земли
                    earth_angular_radius = math.asin(earth_radius / (earth_radius + altitude))
                    
                    # Фактор видимости
                    view_factor = 0.5 * (1 - math.cos(earth_angular_radius))
                    
                    # Фактор освещённости Земли Солнцем
                    sun_illumination_factor = 1.0 if self.sun_visible else 0.0
                    
                    # Мощность от альбедо
                    albedo_power = (self.solar_constant * self.earth_albedo * 
                                  panel['area'] * self.panel_efficiency *
                                  cos_angle_earth * view_factor * sun_illumination_factor)
                    
                    panel_power += albedo_power
            
            panel_powers[panel_name] = panel_power
            power_details[panel_name] = {
                'direct': direct_power,
                'albedo': albedo_power,
                'total': panel_power
            }
            total_power += panel_power
        
        return total_power, panel_powers, power_details
    
    def _calculate_thermal_balance(self, panel_powers):
        """
        Расчёт теплового баланса с использованием текущей ориентации.
        """
        try:
            # Материальные свойства
            mat_props = self.material_properties['clear_anodized_al']
            solar_abs = mat_props['solar_absorptivity']
            ir_emiss = mat_props['infrared_emissivity']
            
            # Площади граней
            face_areas = {
                '+X': 0.01, '-X': 0.01,
                '+Y': 0.03, '-Y': 0.03,
                '+Z': 0.03, '-Z': 0.03
            }
            
            # Инициализируем тепловые потоки
            heat_fluxes = {}
            new_surface_temps = {}
            
            # Матрица вращения
            rot_body_to_eci = Rotation.from_quat(self.attitude_quat)
            R_body_to_eci = rot_body_to_eci.as_matrix()
            
            # Расчёт для каждой грани
            for face_name, area in face_areas.items():
                current_temp = self.surface_temperatures[face_name]
                normal_body = self.solar_panels[face_name]['normal']
                normal_eci = R_body_to_eci @ normal_body
                
                # 1. ПРЯМОЕ СОЛНЕЧНОЕ ИЗЛУЧЕНИЕ
                solar_flux = 0.0
                if self.sun_visible:
                    cos_angle_sun = np.dot(normal_eci, self.sun_direction_eci)
                    if cos_angle_sun > 0:
                        solar_flux = self.solar_constant * cos_angle_sun * solar_abs
                
                # 2. АЛЬБЕДО ЗЕМЛИ
                albedo_flux = 0.0
                if self.earth_visible and self.sun_visible:
                    earth_direction = -self.pos_eci_km / np.linalg.norm(self.pos_eci_km)
                    cos_angle_earth = np.dot(normal_eci, earth_direction)
                    
                    if cos_angle_earth > 0:
                        earth_radius = 6378.137
                        altitude = np.linalg.norm(self.pos_eci_km) - earth_radius
                        earth_angular_radius = math.asin(earth_radius / (earth_radius + altitude))
                        view_factor = 0.5 * (1 - math.cos(earth_angular_radius))
                        
                        albedo_flux = (self.solar_constant * self.earth_albedo * 
                                     solar_abs * cos_angle_earth * view_factor)
                
                # 3. ИНФРАКРАСНОЕ ИЗЛУЧЕНИЕ ЗЕМЛИ
                earth_ir_flux = 0.0
                if self.earth_visible:
                    earth_direction = -self.pos_eci_km / np.linalg.norm(self.pos_eci_km)
                    cos_angle_earth = np.dot(normal_eci, earth_direction)
                    
                    if cos_angle_earth > 0:
                        earth_ir_flux = self.earth_ir_flux * solar_abs * cos_angle_earth
                
                # 4. СОБСТВЕННОЕ ИЗЛУЧЕНИЕ
                emitted_flux = ir_emiss * self.stefan_boltzmann * (current_temp**4)
                
                # 5. ТЕПЛОПРОВОДНОСТЬ
                k = mat_props['thermal_conductivity']
                thickness = mat_props['thickness']
                conductive_flux = (k / thickness) * (self.internal_temperature - current_temp)
                
                # 6. НАГРЕВ ОТ СОЛНЕЧНЫХ ПАНЕЛЕЙ
                panel_heat_flux = 0.0
                if self.solar_panels[face_name]['has_panel'] and face_name in panel_powers:
                    panel_power = panel_powers[face_name]
                    panel_heat = panel_power * (1 - self.panel_efficiency) / self.panel_efficiency
                    panel_heat_flux = panel_heat / area if area > 0 else 0
                
                # СУММАРНЫЙ ПОТОК
                total_flux = (solar_flux + albedo_flux + earth_ir_flux - 
                            emitted_flux + conductive_flux + panel_heat_flux)
                
                # Сохраняем потоки
                heat_fluxes[face_name] = {
                    'solar': solar_flux,
                    'albedo': albedo_flux,
                    'earth_ir': earth_ir_flux,
                    'emitted': emitted_flux,
                    'conductive': conductive_flux,
                    'panel_heat': panel_heat_flux,
                    'total': total_flux
                }
                
                # Обновляем температуру
                delta_time = 1.0
                face_mass = area * thickness * mat_props['density']
                face_heat_capacity = face_mass * mat_props['specific_heat']
                
                if face_heat_capacity > 0:
                    delta_temp = (total_flux * area * delta_time) / face_heat_capacity
                    new_temp = current_temp + delta_temp
                    new_temp = max(100.0, min(400.0, new_temp))
                    new_surface_temps[face_name] = new_temp
                else:
                    new_surface_temps[face_name] = current_temp
            
            # Обновляем внутреннюю температуру
            delta_time = 1.0
            total_surface_flux = sum([flux['conductive'] * face_areas[face] 
                                      for face, flux in heat_fluxes.items()])
            
            internal_heat = self.internal_heat_sources['total']
            delta_temp_internal = ((total_surface_flux + internal_heat) * delta_time) / self.heat_capacity
            new_internal_temp = self.internal_temperature + delta_temp_internal
            new_internal_temp = max(250.0, min(350.0, new_internal_temp))
            
            return new_internal_temp, new_surface_temps, heat_fluxes
            
        except Exception as e:
            print(f"Ошибка при расчёте теплового баланса: {e}")
            return self.internal_temperature, self.surface_temperatures, {}
    
    def _transform_magnetic_field_to_body_frame(self, B_ned_nT, lat_rad, lon_rad):
        """Преобразует вектор магнитного поля в связанную систему."""
        try:
            R_ned_to_ecef = self._calculate_rotation_matrix_ned_to_ecef(lat_rad, lon_rad)
            B_ecef_nT = R_ned_to_ecef @ B_ned_nT
            
            B_eci_nT = self._current_rotation_matrix_ecef_to_eci @ B_ecef_nT
            
            rot_body_to_eci = Rotation.from_quat(self.attitude_quat)
            R_eci_to_body = rot_body_to_eci.as_matrix().T
            
            B_body_nT = R_eci_to_body @ B_eci_nT
            
            return B_body_nT
            
        except Exception as e:
            print(f"Ошибка при преобразовании магнитного поля: {e}")
            return np.array([0.0, 0.0, 0.0])
    
    def update(self, dt_seconds=1.0):
        """Основной цикл обновления состояния эмулятора."""
        try:
            # 1. Обновляем время
            self.current_time = self.current_time + timedelta(seconds=dt_seconds)
            if self.current_time.tzinfo is None:
                self.current_time = self.current_time.replace(tzinfo=utc)
            
            # 2. Вычисляем положение и скорость
            t = self.ts.utc(self.current_time)
            geocentric = self.sat_obj.at(t)
            self.pos_eci_km = geocentric.position.km
            self.vel_eci_km_s = geocentric.velocity.km_per_s
            
            # 3. Получаем и сохраняем геодезические координаты
            subpoint = geocentric.subpoint()
            self.lat_deg = subpoint.latitude.degrees
            self.lon_deg = subpoint.longitude.degrees
            self.alt_km = subpoint.elevation.km
            
            if math.isnan(self.lat_deg) or math.isnan(self.lon_deg) or math.isnan(self.alt_km):
                return False
            
            # 4. Вычисляем GMST и матрицу вращения
            self._current_gmst = self._calculate_gmst(self.current_time)
            self._current_rotation_matrix_ecef_to_eci = self._get_rotation_matrix_ecef_to_eci(self._current_gmst)
            
            # 5. Магнитное поле
            lat_rad = math.radians(self.lat_deg)
            lon_rad = math.radians(self.lon_deg)
            B_ned_nT = self._calculate_magnetic_field(self.lat_deg, self.lon_deg, self.alt_km, self.current_time)
            self.magnetic_field_body = self._transform_magnetic_field_to_body_frame(B_ned_nT, lat_rad, lon_rad)
            
            # 6. Рассчитываем положение Солнца и Земли
            (self.sun_direction_body, self.sun_direction_eci, 
             self.earth_direction_body, self.sun_visible, self.earth_visible) = self._calculate_sun_and_earth_positions(t)
            
            # 7. Солнечные батареи с учётом альбедо
            total_power, panel_powers, power_details = self._calculate_solar_power_with_albedo()
            self.solar_power = total_power
            self.panel_powers = panel_powers
            self.power_details = power_details
            
            # 8. Тепловой баланс
            new_internal_temp, new_surface_temps, heat_fluxes = self._calculate_thermal_balance(panel_powers)
            self.internal_temperature = new_internal_temp
            self.surface_temperatures = new_surface_temps
            self.heat_fluxes = heat_fluxes
            
            # 9. Обновляем ориентацию
            if np.linalg.norm(self.angular_vel) > 1e-12:
                angle = np.linalg.norm(self.angular_vel) * dt_seconds
                axis = self.angular_vel / np.linalg.norm(self.angular_vel)
                
                sin_half_angle = math.sin(angle / 2.0)
                cos_half_angle = math.cos(angle / 2.0)
                
                dq = np.array([
                    axis[0] * sin_half_angle,
                    axis[1] * sin_half_angle,
                    axis[2] * sin_half_angle,
                    cos_half_angle
                ])
                
                q0, q1, q2, q3 = self.attitude_quat
                dq0, dq1, dq2, dq3 = dq
                
                self.attitude_quat = np.array([
                    dq3 * q0 + dq0 * q3 + dq1 * q2 - dq2 * q1,
                    dq3 * q1 - dq0 * q2 + dq1 * q3 + dq2 * q0,
                    dq3 * q2 + dq0 * q1 - dq1 * q0 + dq2 * q3,
                    dq3 * q3 - dq0 * q0 - dq1 * q1 - dq2 * q2
                ])
                
                self.attitude_quat = self.attitude_quat / np.linalg.norm(self.attitude_quat)
            
            return True
            
        except Exception as e:
            print(f"Ошибка в методе update: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def generate_telemetry(self):
        """Генерирует телеметрический пакет."""
        total_field = np.linalg.norm(self.magnetic_field_body)
        
        rot = Rotation.from_quat(self.attitude_quat)
        euler_angles = rot.as_euler('zyx', degrees=True)
        
        # Накопленный угол поворота
        elapsed_seconds = (self.current_time - self.start_time).total_seconds()
        accumulated_rotation_x = math.degrees(self.angular_vel[0] * elapsed_seconds) % 360
        accumulated_rotation_y = math.degrees(self.angular_vel[1] * elapsed_seconds) % 360
        accumulated_rotation_z = math.degrees(self.angular_vel[2] * elapsed_seconds) % 360
        
        # Углы направления на Солнце
        sun_angles = {}
        if np.linalg.norm(self.sun_direction_body) > 0:
            sun_dir_norm = self.sun_direction_body / np.linalg.norm(self.sun_direction_body)
            sun_azimuth_body = math.degrees(math.atan2(sun_dir_norm[1], sun_dir_norm[0]))
            sun_elevation_body = math.degrees(math.asin(sun_dir_norm[2]))
            
            sun_angles = {
                'azimuth_body_deg': sun_azimuth_body,
                'elevation_body_deg': sun_elevation_body,
                'visible': self.sun_visible
            }
        
        # Температуры
        internal_temp_c = self.internal_temperature - 273.15
        surface_temps_c = {face: temp - 273.15 for face, temp in self.surface_temperatures.items()}
        
        # Детализация питания
        power_breakdown = {}
        if hasattr(self, 'power_details'):
            for panel, details in self.power_details.items():
                power_breakdown[panel] = {
                    'direct_w': details['direct'],
                    'albedo_w': details['albedo'],
                    'total_w': details['total']
                }
        
        telemetry = {
            'timestamp': self.current_time.isoformat(),
            'position': {
                'eci_km': {
                    'x': float(self.pos_eci_km[0]), 
                    'y': float(self.pos_eci_km[1]), 
                    'z': float(self.pos_eci_km[2])
                },
                'geodetic': {
                    'lat': float(self.lat_deg), 
                    'lon': float(self.lon_deg), 
                    'alt_km': float(self.alt_km)
                }
            },
            'attitude': {
                'quaternion': {
                    'x': float(self.attitude_quat[0]), 
                    'y': float(self.attitude_quat[1]), 
                    'z': float(self.attitude_quat[2]), 
                    'w': float(self.attitude_quat[3])
                },
                'euler_deg': {
                    'yaw': float(euler_angles[0]), 
                    'pitch': float(euler_angles[1]), 
                    'roll': float(euler_angles[2])
                }
            },
            'sun_info': {
                'direction_body': {
                    'x': float(self.sun_direction_body[0]), 
                    'y': float(self.sun_direction_body[1]), 
                    'z': float(self.sun_direction_body[2])
                },
                'angles_deg': sun_angles,
                'visible': self.sun_visible,
                'earth_visible': self.earth_visible
            },
            'power': {
                'solar_total_w': float(self.solar_power),
                'power_breakdown': power_breakdown,
                'battery_voltage_v': 12.0,
                'bus_current_a': self.solar_power / 12.0 if self.solar_power > 0 else 0.0
            },
            'thermal': {
                'internal_temperature_c': float(internal_temp_c),
                'surface_temperatures_c': surface_temps_c,
                'heat_fluxes': {face: {key: float(value) for key, value in fluxes.items()} 
                               for face, fluxes in getattr(self, 'heat_fluxes', {}).items()}
            },
            'magnetic_field': {
                'body_nt': {
                    'x': float(self.magnetic_field_body[0]), 
                    'y': float(self.magnetic_field_body[1]), 
                    'z': float(self.magnetic_field_body[2]), 
                    'total': float(total_field)
                }
            },
            'orbit_number': int((self.current_time - self.start_time).total_seconds() / 5570.0)
        }
        
        return telemetry


def realtime_demo():
    """Демонстрация эмулятора в реальном времени."""
    
    tle_lines = [
        "1 25544U 98067A   25001.00000000  .00000000  00000-0  00000-0 0  9999",
        "2 25544  51.6426  55.3980 0005000  15.5000 350.0000 15.50000000000000"
    ]
    
    print("=" * 80)
    print("ЭМУЛЯТОР CUBESAT 3U С УЧЁТОМ АЛЬБЕДО ЗЕМЛИ")
    print("=" * 80)
    print("Теперь модель учитывает:")
    print("1. Прямое солнечное излучение")
    print("2. Альбедо Земли (отражённую радиацию)")
    print("3. Тепловое ИК излучение Земли")
    print("=" * 80)
    
    print("\nИнициализация эмулятора...")
    emu = SatelliteEmulator(tle_lines[0], tle_lines[1], use_current_time=True)
    
    print("\n" + "=" * 80)
    print("НАЧАЛО ДЕМОНСТРАЦИИ")
    print("=" * 80)
    
    step_counter = 0
    start_demo_time = time.time()
    
    try:
        while True:
            step_counter += 1
            current_demo_time = time.time() - start_demo_time
            
            print(f"\n--- Шаг {step_counter} (время демо: {current_demo_time:.1f} сек) ---")
            
            success = emu.update(dt_seconds=1.0)
            if not success:
                print("Ошибка обновления состояния!")
                break
            
            telemetry = emu.generate_telemetry()
            
            print(f"Время: {telemetry['timestamp']}")
            print(f"Высота: {telemetry['position']['geodetic']['alt_km']:.1f} км")
            print(f"Координаты: Широта={telemetry['position']['geodetic']['lat']:.1f}°, "
                  f"Долгота={telemetry['position']['geodetic']['lon']:.1f}°")
            print(f"Ориентация: Y={telemetry['attitude']['euler_deg']['yaw']:.1f}°, "
                  f"P={telemetry['attitude']['euler_deg']['pitch']:.1f}°, "
                  f"R={telemetry['attitude']['euler_deg']['roll']:.1f}°")
            
            # Информация о Солнце и Земле
            sun_info = telemetry['sun_info']
            print(f"\nОСВЕЩЕНИЕ:")
            print(f"  Солнце: {'ВИДНО' if sun_info['visible'] else 'НЕ ВИДНО'}")
            print(f"  Земля: {'ВИДНА' if sun_info['earth_visible'] else 'НЕ ВИДНА'}")
            
            if sun_info['visible'] and 'angles_deg' in sun_info:
                angles = sun_info['angles_deg']
                print(f"  Направление на Солнце: Азимут={angles['azimuth_body_deg']:.1f}°, "
                      f"Угол места={angles['elevation_body_deg']:.1f}°")
            
            # Энергетика с учётом альбедо
            print(f"\nЭНЕРГЕТИКА (с альбедо):")
            print(f"  Общая мощность: {telemetry['power']['solar_total_w']:.2f} Вт")
            
            if 'power_breakdown' in telemetry['power']:
                total_direct = 0
                total_albedo = 0
                
                for panel, details in telemetry['power']['power_breakdown'].items():
                    if details['total_w'] > 0:
                        print(f"  {panel}: {details['total_w']:.2f} Вт "
                              f"(Солнце: {details['direct_w']:.2f} Вт, "
                              f"Альбедо: {details['albedo_w']:.2f} Вт)")
                        total_direct += details['direct_w']
                        total_albedo += details['albedo_w']
                
                if total_albedo > 0:
                    print(f"  Вклад альбедо: {total_albedo:.2f} Вт ({total_albedo/(total_direct+total_albedo)*100:.1f}%)")
            
            # Тепловое состояние
            print(f"\nТЕПЛОВОЕ СОСТОЯНИЕ:")
            print(f"  Внутренняя: {telemetry['thermal']['internal_temperature_c']:.1f}°C")
            
            surface_temps = telemetry['thermal']['surface_temperatures_c']
            hot_face = max(surface_temps.items(), key=lambda x: x[1])
            cold_face = min(surface_temps.items(), key=lambda x: x[1])
            print(f"  Горячая грань: {hot_face[0]} ({hot_face[1]:.1f}°C)")
            print(f"  Холодная грань: {cold_face[0]} ({cold_face[1]:.1f}°C)")
            
            # Магнитное поле
            mag = telemetry['magnetic_field']['body_nt']
            print(f"\nМАГНИТНОЕ ПОЛЕ:")
            print(f"  X={mag['x']:.0f}, Y={mag['y']:.0f}, Z={mag['z']:.0f} нТл, Итого={mag['total']:.0f} нТл")
            
            # Каждые 10 шагов показываем статистику
            if step_counter % 10 == 0:
                print(f"\n[СТАТИСТИКА]")
                print(f"  Шагов: {step_counter}")
                print(f"  Орбит: {telemetry['orbit_number']}")
            
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\n\n" + "=" * 80)
        print("ДЕМОНСТРАЦИЯ ОСТАНОВЛЕНА")
        print("=" * 80)
        
        final_tm = emu.generate_telemetry()
        demo_duration = time.time() - start_demo_time
        
        print(f"\nИтоги:")
        print(f"  Длительность: {demo_duration:.1f} сек ({demo_duration/60:.1f} мин)")
        print(f"  Шагов: {step_counter}")
        print(f"  Орбит: {final_tm['orbit_number']}")
        
        # Анализ энергоснабжения
        print(f"\nЭНЕРГЕТИЧЕСКИЙ АНАЛИЗ:")
        print(f"  Альбедо Земли даёт дополнительную мощность солнечным батареям")
        print(f"  Максимальная мощность: {max([final_tm['power']['solar_total_w']]):.1f} Вт")
        
        # Тепловой анализ
        print(f"\nТЕПЛОВОЙ АНАЛИЗ:")
        print(f"  Итоговая температура: {final_tm['thermal']['internal_temperature_c']:.1f}°C")


if __name__ == "__main__":
    print("=" * 80)
    print("ЭМУЛЯТОР CUBESAT 3U С ПОЛНОЙ ФИЗИЧЕСКОЙ МОДЕЛЬЮ")
    print("=" * 80)
    print("ВКЛЮЧАЕТ УЧЁТ АЛЬБЕДО ЗЕМЛИ:")
    print("  • Альбедо Земли: 30% от солнечной постоянной")
    print("  • Учёт альбедо в модели питания солнечных батарей")
    print("  • Учёт альбедо в тепловой модели")
    print("  • Реальная ориентация спутника влияет на оба эффекта")
    print("=" * 80)
    
    response = input("\nЗапустить демонстрацию в реальном времени? (y/n): ")
    if response.lower() == 'y':
        print("\nЗапуск через 3 секунды...")
        time.sleep(1)
        print("2...")
        time.sleep(1)
        print("1...")
        time.sleep(1)
        realtime_demo()
