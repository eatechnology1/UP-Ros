<template>
  <LessonContainer>
    <!-- ========================================================================================
         HERO SECTION: THE SENSOR MATRIX
         Visual: Electromagnetic waves + sensor grid aesthetic
         ======================================================================================== -->
    <div class="hero-section text-white q-mb-xl">
      <div class="hero-content">
        <div class="hero-badge"><q-icon name="sensors" /> MÓDULO 6.3: SENSOR SIMULATION</div>
        <h1 class="hero-title">
          Sensores Simulados: <span class="gradient-text">From Noise to Truth</span>
        </h1>
        <p class="hero-subtitle">
          Un sensor perfecto no existe. Todo dato que recibes está contaminado por ruido térmico,
          cuantización ADC y deriva temporal. Aquí aprenderás a
          <strong>modelar la incertidumbre</strong>
          para que tu robot no confunda una sombra con un obstáculo, ni derive 10 metros en un
          pasillo recto.
        </p>

        <div class="hero-stats">
          <div class="stat-item">
            <div class="stat-val">6 Tipos</div>
            <div class="stat-label">Sensores Críticos</div>
          </div>
          <div class="stat-item">
            <div class="stat-val">10-1000 Hz</div>
            <div class="stat-label">Sampling Rates</div>
          </div>
          <div class="stat-item">
            <div class="stat-val">σ² Variance</div>
            <div class="stat-label">Noise Model</div>
          </div>
        </div>
      </div>

      <div class="hero-viz">
        <!-- Sensor Wave Animation -->
        <svg viewBox="0 0 200 200" class="sensor-wave">
          <defs>
            <linearGradient id="waveGrad" x1="0%" y1="0%" x2="100%" y2="0%">
              <stop offset="0%" stop-color="#3b82f6" stop-opacity="0" />
              <stop offset="50%" stop-color="#8b5cf6" stop-opacity="1" />
              <stop offset="100%" stop-color="#3b82f6" stop-opacity="0" />
            </linearGradient>
          </defs>

          <!-- Lidar Rays -->
          <g class="lidar-rays">
            <line
              x1="100"
              y1="100"
              x2="180"
              y2="50"
              stroke="url(#waveGrad)"
              stroke-width="2"
              class="ray r1"
            />
            <line
              x1="100"
              y1="100"
              x2="180"
              y2="100"
              stroke="url(#waveGrad)"
              stroke-width="2"
              class="ray r2"
            />
            <line
              x1="100"
              y1="100"
              x2="180"
              y2="150"
              stroke="url(#waveGrad)"
              stroke-width="2"
              class="ray r3"
            />
          </g>

          <!-- Sensor Core -->
          <circle cx="100" cy="100" r="15" fill="#fbbf24" class="sensor-core" />
          <circle
            cx="100"
            cy="100"
            r="25"
            fill="none"
            stroke="#fbbf24"
            stroke-width="2"
            opacity="0.3"
            class="sensor-ring"
          />
        </svg>
      </div>
    </div>

    <!-- ========================================================================================
         BLOCK A: NOISE THEORY & SENSOR FUNDAMENTALS
         Depth: Stochastic Processes & Signal Processing
         ======================================================================================== -->
    <div class="content-block">
      <SectionTitle>1. Teoría del Ruido Estocástico</SectionTitle>

      <TextBlock>
        Cuando un Lidar reporta "distancia = 5.234m", ese número es una
        <strong>variable aleatoria</strong>, no una verdad absoluta. La medición real podría estar
        entre 5.20m y 5.27m debido a:
        <ul class="q-mt-md">
          <li>
            <strong>Ruido Térmico:</strong> Electrones vibrando en el circuito (Johnson-Nyquist
            Noise).
          </li>
          <li>
            <strong>Cuantización ADC:</strong> El conversor analógico-digital tiene resolución
            finita (ej. 12 bits).
          </li>
          <li>
            <strong>Deriva Temporal:</strong> El sensor se calienta y su offset cambia (Bias Drift).
          </li>
        </ul>
      </TextBlock>

      <!-- GAUSSIAN NOISE VIZ -->
      <div class="noise-theory-container q-my-lg">
        <div class="theory-grid">
          <div class="theory-card">
            <div class="card-header">
              <q-icon name="show_chart" size="lg" />
              <span>Ruido Gaussiano (White Noise)</span>
            </div>
            <div class="card-content">
              <div class="math-expression">n(t) ~ N(0, σ²)</div>
              <div class="card-desc">
                Cada muestra es independiente. La desviación estándar σ define la "nube" de
                incertidumbre. Ejemplo: σ = 0.01m para un Lidar de alta calidad.
              </div>
              <!-- Mini Gaussian Curve -->
              <div class="gaussian-viz">
                <svg viewBox="0 0 100 50" class="bell-curve">
                  <path d="M 10,45 Q 50,5 90,45" fill="none" stroke="#3b82f6" stroke-width="2" />
                  <line
                    x1="50"
                    y1="5"
                    x2="50"
                    y2="45"
                    stroke="#ef4444"
                    stroke-width="1"
                    stroke-dasharray="2,2"
                  />
                  <text x="50" y="48" text-anchor="middle" fill="#94a3b8" font-size="6">μ</text>
                </svg>
              </div>
            </div>
          </div>

          <div class="theory-card">
            <div class="card-header">
              <q-icon name="trending_up" size="lg" />
              <span>Random Walk (Brownian)</span>
            </div>
            <div class="card-content">
              <div class="math-expression">x(t+1) = x(t) + n(t)</div>
              <div class="card-desc">
                Integración de ruido blanco. Causa deriva acumulativa en IMUs (el robot "piensa" que
                está rotando cuando está quieto). Modelado con Allan Variance.
              </div>
              <!-- Random Walk Path -->
              <div class="walk-viz">
                <svg viewBox="0 0 100 50" class="walk-path">
                  <polyline
                    points="10,25 20,22 30,28 40,24 50,30 60,26 70,32 80,28 90,35"
                    fill="none"
                    stroke="#a855f7"
                    stroke-width="2"
                  />
                </svg>
              </div>
            </div>
          </div>
        </div>
      </div>

      <!-- INTERACTIVE NOISE GENERATOR -->
      <div class="tool-wrapper q-mt-xl">
        <div class="tool-header">
          <q-icon name="tune" />
          <div class="tool-title">Noise Generator (Interactive)</div>
        </div>

        <div class="noise-lab">
          <div class="lab-controls">
            <div class="control-group">
              <div class="c-label">Desviación Estándar (σ)</div>
              <q-slider v-model="noiseSigma" :min="0" :max="5" :step="0.1" label color="blue-4" />
              <div class="c-value">σ = {{ noiseSigma.toFixed(1) }}</div>
            </div>

            <div class="control-group">
              <div class="c-label">Bias (Offset)</div>
              <q-slider
                v-model="noiseBias"
                :min="-10"
                :max="10"
                :step="0.5"
                label
                color="orange-4"
              />
              <div class="c-value">μ = {{ noiseBias.toFixed(1) }}</div>
            </div>

            <div class="control-group">
              <div class="c-label">Muestras</div>
              <q-slider
                v-model="noiseSamples"
                :min="10"
                :max="100"
                :step="10"
                label
                color="green-4"
              />
              <div class="c-value">N = {{ noiseSamples }}</div>
            </div>
          </div>

          <div class="lab-viz">
            <div class="noise-plot">
              <div class="plot-title">Distribución de Mediciones</div>
              <div class="plot-canvas">
                <div
                  v-for="(sample, i) in noisyData"
                  :key="i"
                  class="sample-bar"
                  :style="{
                    height: `${Math.abs(sample) * 3}px`,
                    left: `${(i / noiseSamples) * 100}%`,
                    background: sample > 0 ? '#3b82f6' : '#ef4444',
                  }"
                ></div>
                <div class="zero-line"></div>
              </div>
              <div class="plot-stats">
                <span>Mean: {{ computedMean.toFixed(2) }}</span>
                <span>Std Dev: {{ computedStd.toFixed(2) }}</span>
              </div>
            </div>
          </div>
        </div>
      </div>

      <!-- CODE: GAUSSIAN NOISE MODEL -->
      <div class="code-section q-mt-xl">
        <div class="code-header">
          <q-icon name="code" />
          <span>GaussianNoiseModel.cpp (C++ Implementation)</span>
        </div>
        <CodeBlock
          lang="cpp"
          :copyable="true"
          title="Modelo de Ruido para Sensores Gazebo"
          content='#include <random>
#include <gazebo/sensors/SensorTypes.hh>

namespace gazebo {
  class GaussianNoiseModel : public sensors::Noise {
    public:
      GaussianNoiseModel() : generator_(std::random_device{}()) {}

      // Aplica ruido gaussiano a la medición
      virtual double Apply(double _in) override {
        // Distribución normal con media=0 y desviación=sigma
        std::normal_distribution<double> dist(bias_, sigma_);
        double noise = dist(generator_);

        return _in + noise; // Medición real + ruido
      }

      // Configuración desde SDF
      virtual void Load(sdf::ElementPtr _sdf) override {
        if (_sdf->HasElement("mean")) {
          bias_ = _sdf->Get<double>("mean");
        }
        if (_sdf->HasElement("stddev")) {
          sigma_ = _sdf->Get<double>("stddev");
        }

        printf("Noise Model Loaded: μ=%.3f, σ=%.3f\\n", bias_, sigma_);
      }

    private:
      double bias_ = 0.0;   // Offset constante
      double sigma_ = 0.01; // Desviación estándar
      std::mt19937 generator_; // Mersenne Twister RNG
  };

  GZ_REGISTER_SENSOR_NOISE(GaussianNoiseModel)
}'
        />
      </div>
    </div>

    <!-- SECTION 2: SENSOR CLASSIFICATION -->
    <div class="content-block q-mt-xl">
      <SectionTitle>2. Clasificación de Sensores</SectionTitle>

      <TextBlock>
        Los sensores se dividen en dos categorías fundamentales según qué miden:
      </TextBlock>

      <div class="classification-grid q-my-lg">
        <div class="class-card extero">
          <div class="class-icon">
            <q-icon name="radar" size="xl" />
          </div>
          <div class="class-title">Exteroceptivos</div>
          <div class="class-desc">
            Miden el <strong>entorno externo</strong>. Responden a estímulos fuera del robot.
          </div>
          <div class="class-examples">
            <div class="example-tag">Lidar</div>
            <div class="example-tag">Cámara RGB</div>
            <div class="example-tag">Depth Camera</div>
            <div class="example-tag">Ultrasónico</div>
          </div>
          <div class="class-use">
            <strong>Uso:</strong> Detección de obstáculos, SLAM, reconocimiento de objetos.
          </div>
        </div>

        <div class="class-card proprio">
          <div class="class-icon">
            <q-icon name="speed" size="xl" />
          </div>
          <div class="class-title">Propioceptivos</div>
          <div class="class-desc">
            Miden el <strong>estado interno</strong> del robot (posición, velocidad, orientación).
          </div>
          <div class="class-examples">
            <div class="example-tag">IMU</div>
            <div class="example-tag">Encoders</div>
            <div class="example-tag">GPS</div>
            <div class="example-tag">Force/Torque</div>
          </div>
          <div class="class-use">
            <strong>Uso:</strong> Odometría, estabilización, localización absoluta.
          </div>
        </div>
      </div>

      <!-- SAMPLING FREQUENCY TABLE -->
      <div class="freq-table-container q-mt-lg">
        <div class="table-title">Frecuencias de Muestreo Típicas</div>
        <table class="freq-table">
          <thead>
            <tr>
              <th>Sensor</th>
              <th>Frecuencia</th>
              <th>Latencia</th>
              <th>Caso de Uso</th>
            </tr>
          </thead>
          <tbody>
            <tr>
              <td><q-icon name="explore" color="blue" /> IMU</td>
              <td><code>100-1000 Hz</code></td>
              <td>~1 ms</td>
              <td>Estabilización de drones, detección de colisiones</td>
            </tr>
            <tr>
              <td><q-icon name="videocam" color="purple" /> Cámara</td>
              <td><code>30-60 FPS</code></td>
              <td>~16 ms</td>
              <td>Visión por computador, detección de líneas</td>
            </tr>
            <tr>
              <td><q-icon name="radar" color="green" /> Lidar 2D</td>
              <td><code>10-40 Hz</code></td>
              <td>~25 ms</td>
              <td>Navegación indoor, mapeo 2D</td>
            </tr>
            <tr>
              <td><q-icon name="3d_rotation" color="amber" /> Lidar 3D</td>
              <td><code>5-20 Hz</code></td>
              <td>~50 ms</td>
              <td>Mapeo outdoor, vehículos autónomos</td>
            </tr>
            <tr>
              <td><q-icon name="settings_ethernet" color="orange" /> Encoders</td>
              <td><code>50-200 Hz</code></td>
              <td>~5 ms</td>
              <td>Odometría de ruedas, control de motores</td>
            </tr>
          </tbody>
        </table>
      </div>

      <!-- SYNCHRONIZATION TIMELINE VIZ -->
      <div class="sync-timeline q-mt-xl">
        <div class="timeline-header">
          <q-icon name="schedule" />
          <span>Timeline de Sincronización Multi-Sensor</span>
        </div>
        <div class="timeline-canvas">
          <div class="timeline-track imu">
            <div class="track-label">IMU (200Hz)</div>
            <div class="track-pulses">
              <div v-for="n in 20" :key="n" class="pulse" :style="{ left: `${n * 5}%` }"></div>
            </div>
          </div>
          <div class="timeline-track lidar">
            <div class="track-label">Lidar (20Hz)</div>
            <div class="track-pulses">
              <div v-for="n in 4" :key="n" class="pulse" :style="{ left: `${n * 25}%` }"></div>
            </div>
          </div>
          <div class="timeline-track camera">
            <div class="track-label">Camera (30Hz)</div>
            <div class="track-pulses">
              <div v-for="n in 6" :key="n" class="pulse" :style="{ left: `${n * 16.6}%` }"></div>
            </div>
          </div>
        </div>
        <div class="timeline-note">
          <strong>Nota:</strong> La sincronización temporal es crítica para fusión de sensores. ROS
          2 usa <code>message_filters::ApproximateTime</code> para alinear datos con timestamps
          diferentes.
        </div>
      </div>
    </div>

    <!-- ========================================================================================
         BLOCK B: EXTEROCEPTIVE SENSORS (LIDAR & CAMERAS)
         Depth: Ray Casting & Computer Vision
         ======================================================================================== -->
    <div class="section-divider">
      <q-separator spaced="lg" color="indigo-9" />
    </div>

    <!-- SECTION 3: LIDAR SIMULATION -->
    <div class="content-block">
      <SectionTitle>3. Lidar Simulation: Ray Casting Engine</SectionTitle>

      <TextBlock>
        Un Lidar (Light Detection And Ranging) emite pulsos láser y mide el tiempo de retorno para
        calcular distancias. En simulación, Gazebo usa <strong>Ray Casting</strong>: lanza rayos
        virtuales desde el sensor y detecta intersecciones con la geometría del mundo. Para alta
        performance, se usa <strong>GPU Ray</strong> (CUDA).
      </TextBlock>

      <!-- LIDAR CONFIGURATOR -->
      <div class="lidar-config-tool q-my-lg">
        <div class="tool-header">
          <q-icon name="radar" />
          <div class="tool-title">Lidar Configurator</div>
        </div>
        <div class="config-grid">
          <div class="config-controls">
            <div class="control-group">
              <div class="c-label">Horizontal FOV (°)</div>
              <q-slider v-model="lidarFOV" :min="90" :max="360" :step="10" label color="green-4" />
              <div class="c-value">{{ lidarFOV }}°</div>
            </div>
            <div class="control-group">
              <div class="c-label">Resolution (rays)</div>
              <q-slider
                v-model="lidarRays"
                :min="180"
                :max="1080"
                :step="60"
                label
                color="blue-4"
              />
              <div class="c-value">{{ lidarRays }} rays</div>
            </div>
            <div class="control-group">
              <div class="c-label">Max Range (m)</div>
              <q-slider v-model="lidarRange" :min="5" :max="30" :step="1" label color="purple-4" />
              <div class="c-value">{{ lidarRange }}m</div>
            </div>
          </div>
          <div class="config-viz">
            <div class="lidar-scan-viz">
              <svg viewBox="0 0 200 200" class="scan-canvas">
                <circle cx="100" cy="100" r="5" fill="#22c55e" />
                <g v-for="i in Math.floor(lidarRays / 10)" :key="i">
                  <line
                    :x1="100"
                    :y1="100"
                    :x2="
                      100 +
                      Math.cos((i / Math.floor(lidarRays / 10)) * ((lidarFOV * Math.PI) / 180)) *
                        (lidarRange * 3)
                    "
                    :y2="
                      100 +
                      Math.sin((i / Math.floor(lidarRays / 10)) * ((lidarFOV * Math.PI) / 180)) *
                        (lidarRange * 3)
                    "
                    stroke="#22c55e"
                    stroke-width="1"
                    opacity="0.3"
                  />
                </g>
              </svg>
              <div class="viz-stats">
                <span>Angular Resolution: {{ (lidarFOV / lidarRays).toFixed(3) }}°/ray</span>
              </div>
            </div>
          </div>
        </div>
      </div>

      <!-- LIDAR SDF CODE -->
      <div class="code-section">
        <div class="code-header">
          <q-icon name="code" />
          <span>lidar_sensor.sdf (GPU Ray Configuration)</span>
        </div>
        <CodeBlock
          lang="xml"
          :copyable="true"
          title="Configuración Completa de Lidar 2D"
          content='<sensor name="lidar" type="gpu_ray">
  <pose>0 0 0.1 0 0 0</pose>
  <visualize>true</visualize>
  <update_rate>20</update_rate> <!-- 20 Hz típico para navegación indoor -->

  <ray>
    <scan>
      <!-- HORIZONTAL SCAN (2D Lidar) -->
      <horizontal>
        <samples>720</samples> <!-- Número de rayos -->
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle> <!-- -180° -->
        <max_angle>3.14159</max_angle>  <!-- +180° -->
      </horizontal>
    </scan>

    <range>
      <min>0.10</min> <!-- Blind spot: 10cm -->
      <max>30.0</max> <!-- Max range: 30m -->
      <resolution>0.01</resolution> <!-- 1cm precision -->
    </range>

    <!-- NOISE MODEL (Gaussian) -->
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev> <!-- σ = 1cm (High-quality Lidar) -->
    </noise>
  </ray>

  <!-- ROS 2 PLUGIN -->
  <plugin name="gazebo_ros_laser" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
    <frame_name>lidar_link</frame_name>
  </plugin>
</sensor>'
        />
      </div>
    </div>

    <!-- SECTION 4: DEPTH CAMERAS (RGBD) -->
    <div class="content-block q-mt-xl">
      <SectionTitle>4. Depth Cameras: RGBD Simulation</SectionTitle>

      <TextBlock>
        Las cámaras de profundidad (Intel RealSense, Kinect) combinan imagen RGB con un mapa de
        distancias (Depth). Existen 3 tecnologías: <strong>Stereo Vision</strong> (2 cámaras),
        <strong>Structured Light</strong> (patrón IR), y <strong>Time-of-Flight</strong> (ToF).
        Gazebo simula el resultado final: una Point Cloud 3D.
      </TextBlock>

      <!-- CAMERA FRUSTUM VIZ -->
      <div class="camera-frustum-viz q-my-lg">
        <div class="frustum-container">
          <div class="frustum-3d">
            <div class="frustum-pyramid">
              <div class="frustum-face front"></div>
              <div class="frustum-face back"></div>
              <div class="frustum-face left"></div>
              <div class="frustum-face right"></div>
              <div class="frustum-face top"></div>
              <div class="frustum-face bottom"></div>
            </div>
          </div>
          <div class="frustum-params">
            <div class="param-row">
              <span class="param-label">FOV Horizontal:</span>
              <code>69.4°</code>
            </div>
            <div class="param-row">
              <span class="param-label">FOV Vertical:</span>
              <code>42.5°</code>
            </div>
            <div class="param-row">
              <span class="param-label">Near Clip:</span>
              <code>0.3m</code>
            </div>
            <div class="param-row">
              <span class="param-label">Far Clip:</span>
              <code>10.0m</code>
            </div>
          </div>
        </div>
      </div>

      <!-- RGBD CAMERA CODE -->
      <div class="code-section">
        <div class="code-header">
          <q-icon name="code" />
          <span>rgbd_camera.sdf (Depth + RGB)</span>
        </div>
        <CodeBlock
          lang="xml"
          :copyable="true"
          title="Configuración de Cámara RGBD (Intel RealSense D435)"
          content='<sensor name="rgbd_camera" type="depth">
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.211</horizontal_fov> <!-- 69.4° -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.3</near>
      <far>10.0</far>
    </clip>

    <!-- DISTORTION (Lens imperfection) -->
    <distortion>
      <k1>0.0</k1>
      <k2>0.0</k2>
      <k3>0.0</k3>
      <p1>0.0</p1>
      <p2>0.0</p2>
      <center>0.5 0.5</center>
    </distortion>

    <!-- NOISE ON DEPTH CHANNEL -->
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev> <!-- 7mm error típico -->
    </noise>
  </camera>

  <!-- ROS 2 PLUGIN (Publica RGB + Depth + PointCloud) -->
  <plugin name="camera_driver" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/camera</namespace>
      <remapping>~/image_raw:=rgb/image_raw</remapping>
      <remapping>~/depth/image_raw:=depth/image_raw</remapping>
      <remapping>~/points:=depth/points</remapping>
    </ros>
    <camera_name>rgbd</camera_name>
    <frame_name>camera_link</frame_name>
    <hack_baseline>0.07</hack_baseline> <!-- Stereo baseline -->
  </plugin>
</sensor>'
        />
      </div>
    </div>

    <!-- ========================================================================================
         BLOCK C: PROPRIOCEPTIVE SENSORS (IMU & ODOMETRY)
         Depth: Inertial Navigation & Dead Reckoning
         ======================================================================================== -->
    <div class="section-divider">
      <q-separator spaced="lg" color="indigo-9" />
    </div>

    <!-- SECTION 5: IMU SIMULATION -->
    <div class="content-block">
      <SectionTitle>5. IMU: Inertial Measurement Unit</SectionTitle>

      <TextBlock>
        Una IMU combina 3 sensores: <strong>Acelerómetro</strong> (aceleración lineal),
        <strong>Giroscopio</strong> (velocidad angular), y opcionalmente
        <strong>Magnetómetro</strong> (orientación absoluta). El problema crítico es el
        <strong>Bias Drift</strong>: el sensor reporta movimiento cuando está quieto.
      </TextBlock>

      <!-- IMU COMPONENTS VIZ -->
      <div class="imu-components-grid q-my-lg">
        <div class="imu-component accel">
          <q-icon name="trending_up" size="xl" />
          <div class="comp-title">Acelerómetro</div>
          <div class="comp-desc">
            Mide aceleración en 3 ejes (m/s²). Incluye gravedad (9.81 m/s² en Z).
          </div>
          <div class="comp-noise">Noise: <code>σ = 0.01 m/s²</code></div>
        </div>
        <div class="imu-component gyro">
          <q-icon name="360" size="xl" />
          <div class="comp-title">Giroscopio</div>
          <div class="comp-desc">
            Mide velocidad angular (rad/s). Integrar da orientación, pero deriva con el tiempo.
          </div>
          <div class="comp-noise">Bias Drift: <code>0.001 rad/s/s</code></div>
        </div>
        <div class="imu-component mag">
          <q-icon name="explore" size="xl" />
          <div class="comp-title">Magnetómetro</div>
          <div class="comp-desc">
            Mide campo magnético terrestre. Provee orientación absoluta (como una brújula).
          </div>
          <div class="comp-noise">Interferencia: <code>Metales cercanos</code></div>
        </div>
      </div>

      <!-- IMU SDF CODE -->
      <div class="code-section">
        <div class="code-header">
          <q-icon name="code" />
          <span>imu_sensor.sdf (9-DOF IMU)</span>
        </div>
        <CodeBlock
          lang="xml"
          :copyable="true"
          title="Configuración Realista de IMU (MPU-9250)"
          content='<sensor name="imu" type="imu">
  <always_on>true</always_on>
  <update_rate>200</update_rate> <!-- 200 Hz para control de drones -->

  <imu>
    <!-- ANGULAR VELOCITY (Gyroscope) -->
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.009</stddev> <!-- White noise -->
          <bias_mean>0.00075</bias_mean> <!-- Bias drift -->
          <bias_stddev>0.0000008</bias_stddev>
        </noise>
      </x>
      <y> <!-- Same for Y and Z --> </y>
      <z> <!-- Same for Y and Z --> </z>
    </angular_velocity>

    <!-- LINEAR ACCELERATION (Accelerometer) -->
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.1</bias_mean>
          <bias_stddev>0.001</bias_stddev>
        </noise>
      </x>
      <y> <!-- Same --> </y>
      <z> <!-- Same --> </z>
    </linear_acceleration>
  </imu>

  <!-- ROS 2 PLUGIN -->
  <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>~/out:=imu/data</remapping>
    </ros>
    <frame_name>imu_link</frame_name>
    <initial_orientation_as_reference>false</initial_orientation_as_reference>
  </plugin>
</sensor>'
        />
      </div>
    </div>

    <!-- SECTION 6: WHEEL ODOMETRY -->
    <div class="content-block q-mt-xl">
      <SectionTitle>6. Wheel Odometry: Dead Reckoning</SectionTitle>

      <TextBlock>
        Los encoders de rueda miden rotaciones y calculan la distancia recorrida. El modelo
        cinemático
        <strong>diferencial</strong> (2 ruedas) convierte velocidades de rueda en velocidad del
        robot. El error principal es el <strong>slip</strong> (deslizamiento en superficies lisas).
      </TextBlock>

      <!-- DIFF DRIVE MODEL VIZ -->
      <div class="diff-drive-viz q-my-lg">
        <div class="drive-diagram">
          <svg viewBox="0 0 200 150" class="robot-top-view">
            <!-- Robot Body -->
            <rect
              x="60"
              y="50"
              width="80"
              height="50"
              fill="#1e293b"
              stroke="#3b82f6"
              stroke-width="2"
              rx="5"
            />
            <!-- Left Wheel -->
            <rect x="50" y="60" width="10" height="30" fill="#22c55e" />
            <text x="45" y="80" fill="#22c55e" font-size="10">ωL</text>
            <!-- Right Wheel -->
            <rect x="140" y="60" width="10" height="30" fill="#ef4444" />
            <text x="155" y="80" fill="#ef4444" font-size="10">ωR</text>
            <!-- Center -->
            <circle cx="100" cy="75" r="3" fill="#fbbf24" />
            <!-- Velocity Vector -->
            <line
              x1="100"
              y1="75"
              x2="100"
              y2="30"
              stroke="#a855f7"
              stroke-width="2"
              marker-end="url(#arrowhead)"
            />
            <text x="105" y="50" fill="#a855f7" font-size="10">v</text>
          </svg>
        </div>
        <div class="drive-equations">
          <div class="eq-title">Modelo Cinemático</div>
          <div class="eq-row">
            <code>v = (ωL + ωR) × r / 2</code>
            <span>Velocidad lineal</span>
          </div>
          <div class="eq-row">
            <code>ω = (ωR - ωL) × r / L</code>
            <span>Velocidad angular</span>
          </div>
          <div class="eq-params">
            <div><code>r</code> = Radio de rueda</div>
            <div><code>L</code> = Distancia entre ruedas</div>
          </div>
        </div>
      </div>

      <!-- DIFF DRIVE PLUGIN CODE -->
      <div class="code-section">
        <div class="code-header">
          <q-icon name="code" />
          <span>diff_drive.sdf (Plugin con Slip)</span>
        </div>
        <CodeBlock
          lang="xml"
          :copyable="true"
          title="Differential Drive con Ruido de Odometría"
          content='<plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
  <ros>
    <namespace>/robot</namespace>
    <remapping>cmd_vel:=cmd_vel</remapping>
    <remapping>odom:=odom</remapping>
  </ros>

  <!-- WHEEL JOINTS -->
  <left_joint>left_wheel_joint</left_joint>
  <right_joint>right_wheel_joint</right_joint>

  <!-- KINEMATICS -->
  <wheel_separation>0.4</wheel_separation> <!-- L = 40cm -->
  <wheel_diameter>0.1</wheel_diameter>     <!-- r = 5cm -->

  <!-- ODOMETRY NOISE (Slip simulation) -->
  <odometry_source>world</odometry_source> <!-- "encoder" for realistic drift -->
  <publish_odom>true</publish_odom>
  <publish_odom_tf>true</publish_odom_tf>
  <publish_wheel_tf>false</publish_wheel_tf>

  <!-- COVARIANCE (Para EKF fusion) -->
  <covariance_x>0.0001</covariance_x>
  <covariance_y>0.0001</covariance_y>
  <covariance_yaw>0.01</covariance_yaw>

  <frame_name>odom</frame_name>
  <child_frame_name>base_footprint</child_frame_name>
</plugin>'
        />
      </div>
    </div>

    <!-- ========================================================================================
         BLOCK D: CASE STUDY & SUMMARY
         Depth: Integration & Best Practices
         ======================================================================================== -->
    <div class="section-divider">
      <q-separator spaced="lg" color="indigo-9" />
    </div>

    <!-- SECTION 7: CASE STUDY - TITAN SENSOR SUITE -->
    <div class="content-block">
      <SectionTitle>7. Case Study: Titan Sensor Suite</SectionTitle>
      <TextBlock>
        Hemos integrado todos los sensores en un robot móvil autónomo llamado
        <strong>"Titan"</strong>. Este sistema combina 6 sensores para navegación robusta en
        entornos industriales complejos.
      </TextBlock>

      <!-- SENSOR SUITE VIZ -->
      <div class="sensor-suite-viz q-my-lg">
        <div class="suite-header">
          <q-icon name="precision_manufacturing" size="xl" color="amber" />
          <div class="suite-title">Titan Sensor Configuration</div>
        </div>
        <div class="suite-grid">
          <div class="sensor-card lidar-card">
            <q-icon name="radar" size="lg" />
            <div class="sensor-name">2x Lidar 2D</div>
            <div class="sensor-spec">Front: 270° FOV, Rear: 180° FOV</div>
            <div class="sensor-freq">20 Hz</div>
          </div>
          <div class="sensor-card camera-card">
            <q-icon name="videocam" size="lg" />
            <div class="sensor-name">1x RGBD Camera</div>
            <div class="sensor-spec">640x480, Depth 0.3-10m</div>
            <div class="sensor-freq">30 FPS</div>
          </div>
          <div class="sensor-card imu-card">
            <q-icon name="explore" size="lg" />
            <div class="sensor-name">1x IMU (9-DOF)</div>
            <div class="sensor-spec">Accel + Gyro + Mag</div>
            <div class="sensor-freq">200 Hz</div>
          </div>
          <div class="sensor-card encoder-card">
            <q-icon name="settings_ethernet" size="lg" />
            <div class="sensor-name">2x Wheel Encoders</div>
            <div class="sensor-spec">Differential Drive</div>
            <div class="sensor-freq">100 Hz</div>
          </div>
        </div>

        <!-- METRICS -->
        <div class="suite-metrics">
          <div class="metric-item">
            <div class="metric-label">Total Bandwidth</div>
            <code>~15 MB/s</code>
          </div>
          <div class="metric-item">
            <div class="metric-label">Latency (Avg)</div>
            <code>25 ms</code>
          </div>
          <div class="metric-item">
            <div class="metric-label">CPU Usage</div>
            <code>~40%</code>
          </div>
        </div>
      </div>

      <!-- TITAN URDF CODE -->
      <q-expansion-item
        class="bg-slate-800 text-white rounded-lg border border-slate-700 q-mb-xl"
        icon="description"
        label="Ver URDF Completo: titan_sensors.urdf.xacro"
        header-class="text-amber-400 font-bold"
      >
        <div class="q-pa-md">
          <CodeBlock
            lang="xml"
            content='<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="titan">

  <!-- LIDAR FRONT -->
  <link name="lidar_front_link"/>
  <joint name="lidar_front_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_front_link"/>
    <origin xyz="0.3 0 0.2" rpy="0 0 0"/>
  </joint>

  <gazebo reference="lidar_front_link">
    <sensor name="lidar_front" type="gpu_ray">
      <pose>0 0 0 0 0 0</pose>
      <update_rate>20</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>540</samples>
            <min_angle>-2.356</min_angle> <!-- -135° -->
            <max_angle>2.356</max_angle>  <!-- +135° -->
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
        </range>
        <noise>
          <type>gaussian</type>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="lidar_front_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <remapping>~/out:=scan_front</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>lidar_front_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- RGBD CAMERA -->
  <link name="camera_link"/>
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.25 0 0.3" rpy="0 0 0"/>
  </joint>

  <gazebo reference="camera_link">
    <sensor name="rgbd_camera" type="depth">
      <update_rate>30</update_rate>
      <camera>
        <horizontal_fov>1.211</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
        </image>
        <clip>
          <near>0.3</near>
          <far>10.0</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/camera</namespace>
        </ros>
        <frame_name>camera_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- IMU -->
  <link name="imu_link"/>
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

  <gazebo reference="imu_link">
    <sensor name="imu" type="imu">
      <always_on>true</always_on>
      <update_rate>200</update_rate>
      <plugin name="imu_controller" filename="libgazebo_ros_imu_sensor.so">
        <ros>
          <remapping>~/out:=imu/data</remapping>
        </ros>
        <frame_name>imu_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

</robot>'
            :copyable="true"
          />
        </div>
      </q-expansion-item>
    </div>

    <!-- SECTION 8: DOCTORAL CHALLENGE -->
    <div class="content-block q-mb-xl">
      <SectionTitle>8. Doctor's Challenge: The Sensor Fusion Paradox</SectionTitle>

      <div class="challenge-card">
        <div class="challenge-header">
          <q-icon name="psychology" class="c-icon" />
          <div class="c-info">
            <div class="c-title">🧠 La Paradoja de la Fusión</div>
            <div class="c-desc">
              Tienes un robot con IMU (200Hz) y Lidar (20Hz). Agregas un segundo Lidar para
              redundancia. Después de la fusión con EKF, la localización empeora (error aumenta de
              5cm a 15cm). ¿Por qué?
            </div>
          </div>
        </div>

        <div class="challenge-options">
          <div class="c-option correct">
            <div class="opt-radio">A</div>
            <div class="opt-text">
              La matriz de covarianza del segundo Lidar está mal calibrada (σ² muy pequeña),
              haciendo que el EKF confíe demasiado en datos ruidosos.
              <div class="opt-feedback">
                ¡Correcto! Si reportas σ²=0.01 pero el ruido real es σ²=0.1, el filtro ignora
                sensores buenos. Usa <code>robot_localization</code> con covariance tuning.
              </div>
            </div>
          </div>
          <div class="c-option">
            <div class="opt-radio">B</div>
            <div class="opt-text">Los Lidars interfieren electromagnéticamente entre sí.</div>
          </div>
          <div class="c-option">
            <div class="opt-radio">C</div>
            <div class="opt-text">ROS 2 no soporta múltiples sensores del mismo tipo.</div>
          </div>
        </div>
      </div>
    </div>

    <!-- SECTION 9: VIDEO TUTORIAL -->
    <div class="content-block q-mb-xl">
      <SectionTitle>9. Video Tutorial: Integración de Sensor Suite</SectionTitle>
      <div class="video-container">
        <div class="video-wrapper">
          <iframe
            width="100%"
            height="100%"
            src="https://www.youtube.com/embed/Romc22GgusU"
            title="Sensor Suite Tutorial"
            frameborder="0"
            allow="
              accelerometer;
              autoplay;
              clipboard-write;
              encrypted-media;
              gyroscope;
              picture-in-picture;
            "
            allowfullscreen
          ></iframe>
        </div>
        <div class="video-caption">
          <q-icon name="live_tv" />
          Configuración completa de Titan: URDF, TF tree, sincronización temporal con
          <code>message_filters</code>, y debug de covariance en RViz.
        </div>
      </div>
    </div>

    <!-- SECTION 10: SUMMARY -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen de Ingeniería</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>Ruido Gaussiano</code>
          <span>Modelar con N(μ, σ²). Calibrar σ² experimentalmente, no asumir valores.</span>
        </div>
        <div class="summary-item">
          <code>Lidar GPU Ray</code>
          <span>Usa CUDA para ray casting paralelo. 10x más rápido que CPU ray.</span>
        </div>
        <div class="summary-item">
          <code>RGBD Depth</code>
          <span>Publica 3 topics: RGB, Depth, PointCloud. Usa PointCloud para SLAM 3D.</span>
        </div>
        <div class="summary-item">
          <code>IMU Bias Drift</code>
          <span>Integrar giroscopio causa deriva. Fusionar con magnetómetro o GPS.</span>
        </div>
        <div class="summary-item">
          <code>Wheel Slip</code>
          <span>Odometría deriva en superficies lisas. Combinar con Lidar (AMCL).</span>
        </div>
        <div class="summary-item">
          <code>Covariance Tuning</code>
          <span>Crítico para EKF. Usar Allan Variance para IMUs, experimentos para Lidar.</span>
        </div>
      </div>

      <AlertBlock type="success" title="Best Practices: Sensor Integration" class="q-mt-lg">
        ✅ <strong>TF Tree:</strong> Publicar transforms estáticas de sensores con
        <code>static_transform_publisher</code>.
        <br />
        ✅ <strong>Sincronización:</strong> Usar <code>message_filters::ApproximateTime</code> para
        alinear timestamps.
        <br />
        ✅ <strong>Frecuencias:</strong> IMU > Lidar > Cámara. Nunca fusionar sensores lentos con
        rápidos sin interpolación.
        <br />
        ✅ <strong>Covariance:</strong> Reportar incertidumbre realista. Mejor sobrestimar que
        subestimar.
      </AlertBlock>
    </div>
  </LessonContainer>
</template>

<script setup lang="ts">
import { ref, computed } from 'vue';
import LessonContainer from 'components/content/LessonContainer.vue';
import TextBlock from 'components/content/TextBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';

// --- NOISE GENERATOR LOGIC ---
const noiseSigma = ref(1.0);
const noiseBias = ref(0.0);
const noiseSamples = ref(50);

// --- LIDAR CONFIGURATOR LOGIC ---
const lidarFOV = ref(270);
const lidarRays = ref(720);
const lidarRange = ref(15);

// Generate noisy data based on parameters
const noisyData = computed(() => {
  const data = [];
  for (let i = 0; i < noiseSamples.value; i++) {
    // Box-Muller transform for Gaussian random numbers
    const u1 = Math.random();
    const u2 = Math.random();
    const z = Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
    data.push(noiseBias.value + noiseSigma.value * z);
  }
  return data;
});

const computedMean = computed(() => {
  const sum = noisyData.value.reduce((a, b) => a + b, 0);
  return sum / noisyData.value.length;
});

const computedStd = computed(() => {
  const mean = computedMean.value;
  const variance =
    noisyData.value.reduce((sum, val) => sum + Math.pow(val - mean, 2), 0) / noisyData.value.length;
  return Math.sqrt(variance);
});
</script>

<style scoped>
/* HERO SECTION */
.hero-section {
  background:
    radial-gradient(circle at 20% 80%, rgba(139, 92, 246, 0.15), transparent 50%),
    linear-gradient(135deg, #0f172a 0%, #1e293b 100%);
  border-radius: 24px;
  padding: 4rem 3rem;
  border: 1px solid rgba(148, 163, 184, 0.1);
  box-shadow: 0 20px 50px -10px rgba(0, 0, 0, 0.5);
  display: flex;
  justify-content: space-between;
  align-items: center;
  gap: 4rem;
  overflow: hidden;
  position: relative;
}

.hero-content {
  flex: 1;
  z-index: 2;
}

.hero-badge {
  display: inline-flex;
  align-items: center;
  gap: 0.5rem;
  background: rgba(139, 92, 246, 0.15);
  color: #a78bfa;
  padding: 0.5rem 1rem;
  border-radius: 99px;
  font-size: 0.85rem;
  font-weight: 700;
  border: 1px solid rgba(139, 92, 246, 0.3);
  margin-bottom: 1.5rem;
}

.hero-title {
  font-size: 3.5rem;
  font-weight: 800;
  line-height: 1.1;
  margin-bottom: 1.5rem;
  letter-spacing: -1px;
}

.gradient-text {
  background: linear-gradient(to right, #a78bfa, #22d3ee);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
}

.hero-subtitle {
  font-size: 1.2rem;
  color: #94a3b8;
  margin-bottom: 2.5rem;
  line-height: 1.7;
  max-width: 600px;
}

.hero-stats {
  display: flex;
  gap: 3rem;
}

.stat-val {
  font-size: 1.5rem;
  font-weight: 700;
  color: #fff;
  margin-bottom: 0.25rem;
}

.stat-label {
  font-size: 0.85rem;
  color: #64748b;
  text-transform: uppercase;
  letter-spacing: 1px;
}

/* HERO VIZ */
.hero-viz {
  position: relative;
  width: 400px;
  height: 400px;
  display: flex;
  align-items: center;
  justify-content: center;
}

.sensor-wave {
  width: 100%;
  height: 100%;
}

.sensor-core {
  animation: pulse-sensor 2s infinite;
}

.sensor-ring {
  animation: ring-expand 3s infinite;
}

.ray {
  opacity: 0;
  animation: ray-pulse 3s infinite;
}

.ray.r1 {
  animation-delay: 0s;
}
.ray.r2 {
  animation-delay: 0.5s;
}
.ray.r3 {
  animation-delay: 1s;
}

/* NOISE THEORY */
.noise-theory-container {
  background: rgba(15, 23, 42, 0.6);
  padding: 2rem;
  border-radius: 16px;
  border: 1px solid rgba(148, 163, 184, 0.1);
}

.theory-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 1.5rem;
}

.theory-card {
  background: #0f172a;
  border: 1px solid #334155;
  border-radius: 12px;
  overflow: hidden;
}

.card-header {
  background: #1e293b;
  padding: 1rem;
  display: flex;
  align-items: center;
  gap: 0.75rem;
  border-bottom: 1px solid #334155;
  color: #fff;
  font-weight: 700;
}

.card-content {
  padding: 1.5rem;
}

.math-expression {
  font-family: 'Fira Code', monospace;
  font-size: 1.2rem;
  color: #60a5fa;
  text-align: center;
  padding: 1rem;
  background: rgba(59, 130, 246, 0.1);
  border-radius: 8px;
  margin-bottom: 1rem;
}

.card-desc {
  color: #cbd5e1;
  font-size: 0.9rem;
  line-height: 1.6;
  margin-bottom: 1rem;
}

.gaussian-viz,
.walk-viz {
  background: rgba(0, 0, 0, 0.3);
  border-radius: 8px;
  padding: 0.5rem;
}

/* NOISE LAB */
.tool-wrapper {
  background: #0f172a;
  border-radius: 20px;
  border: 1px solid #334155;
  overflow: hidden;
}

.tool-header {
  background: #1e293b;
  padding: 1rem 1.5rem;
  display: flex;
  align-items: center;
  gap: 0.75rem;
  border-bottom: 1px solid #334155;
}

.tool-header .q-icon {
  font-size: 1.5rem;
  color: #a855f7;
}

.tool-title {
  font-weight: 700;
  color: #fff;
  letter-spacing: 0.5px;
}

.noise-lab {
  display: grid;
  grid-template-columns: 300px 1fr;
  min-height: 400px;
}

.lab-controls {
  padding: 2rem;
  background: rgba(30, 41, 59, 0.5);
  border-right: 1px solid #334155;
  display: flex;
  flex-direction: column;
  gap: 2rem;
}

.control-group {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.c-label {
  font-weight: 700;
  color: #e2e8f0;
  font-size: 0.9rem;
}

.c-value {
  font-family: monospace;
  color: #22d3ee;
  font-size: 1rem;
  font-weight: 700;
}

.lab-viz {
  padding: 2rem;
  background: linear-gradient(180deg, #1e293b 0%, #0f172a 100%);
}

.noise-plot {
  height: 100%;
  display: flex;
  flex-direction: column;
}

.plot-title {
  color: #fff;
  font-weight: 700;
  margin-bottom: 1rem;
}

.plot-canvas {
  flex: 1;
  position: relative;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 8px;
  overflow: hidden;
}

.sample-bar {
  position: absolute;
  bottom: 50%;
  width: 2px;
  transform-origin: bottom;
  opacity: 0.7;
}

.zero-line {
  position: absolute;
  top: 50%;
  left: 0;
  right: 0;
  height: 1px;
  background: #64748b;
}

.plot-stats {
  display: flex;
  justify-content: space-around;
  margin-top: 1rem;
  font-family: monospace;
  color: #94a3b8;
  font-size: 0.9rem;
}

/* CODE SECTION */
.code-section {
  border: 1px solid #334155;
  border-radius: 12px;
  overflow: hidden;
}

.code-header {
  background: #1e293b;
  padding: 0.75rem 1rem;
  display: flex;
  align-items: center;
  gap: 0.75rem;
  border-bottom: 1px solid #334155;
  color: #94a3b8;
  font-family: monospace;
  font-size: 0.9rem;
}

/* CLASSIFICATION */
.classification-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 2rem;
}

.class-card {
  background: rgba(15, 23, 42, 0.6);
  border-radius: 16px;
  padding: 2rem;
  border: 2px solid;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.class-card.extero {
  border-color: #3b82f6;
}

.class-card.proprio {
  border-color: #a855f7;
}

.class-icon {
  color: #fff;
  text-align: center;
}

.class-title {
  font-size: 1.5rem;
  font-weight: 700;
  color: #fff;
  text-align: center;
}

.class-desc {
  color: #cbd5e1;
  text-align: center;
  line-height: 1.6;
}

.class-examples {
  display: flex;
  flex-wrap: wrap;
  gap: 0.5rem;
  justify-content: center;
}

.example-tag {
  background: rgba(255, 255, 255, 0.1);
  padding: 0.25rem 0.75rem;
  border-radius: 99px;
  font-size: 0.8rem;
  color: #e2e8f0;
  border: 1px solid rgba(255, 255, 255, 0.2);
}

.class-use {
  color: #94a3b8;
  font-size: 0.85rem;
  text-align: center;
  padding-top: 1rem;
  border-top: 1px solid rgba(148, 163, 184, 0.2);
}

/* FREQUENCY TABLE */
.freq-table-container {
  background: #0f172a;
  border-radius: 12px;
  overflow: hidden;
  border: 1px solid #334155;
}

.table-title {
  background: #1e293b;
  padding: 1rem;
  font-weight: 700;
  color: #fff;
  border-bottom: 1px solid #334155;
}

.freq-table {
  width: 100%;
  border-collapse: collapse;
}

.freq-table thead {
  background: rgba(30, 41, 59, 0.5);
}

.freq-table th {
  padding: 0.75rem 1rem;
  text-align: left;
  color: #94a3b8;
  font-weight: 600;
  font-size: 0.85rem;
  text-transform: uppercase;
  letter-spacing: 0.5px;
}

.freq-table td {
  padding: 1rem;
  color: #cbd5e1;
  border-top: 1px solid rgba(51, 65, 85, 0.5);
}

.freq-table code {
  background: rgba(59, 130, 246, 0.1);
  padding: 0.25rem 0.5rem;
  border-radius: 4px;
  color: #60a5fa;
  font-family: monospace;
}

/* SYNC TIMELINE */
.sync-timeline {
  background: #0f172a;
  border: 1px solid #334155;
  border-radius: 12px;
  overflow: hidden;
}

.timeline-header {
  background: #1e293b;
  padding: 1rem;
  display: flex;
  align-items: center;
  gap: 0.75rem;
  border-bottom: 1px solid #334155;
  color: #fff;
  font-weight: 700;
}

.timeline-canvas {
  padding: 2rem;
  background: linear-gradient(180deg, #1e293b 0%, #0f172a 100%);
}

.timeline-track {
  margin-bottom: 2rem;
  position: relative;
}

.track-label {
  color: #94a3b8;
  font-size: 0.85rem;
  margin-bottom: 0.5rem;
  font-weight: 600;
}

.track-pulses {
  position: relative;
  height: 30px;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 4px;
}

.pulse {
  position: absolute;
  top: 50%;
  transform: translateY(-50%);
  width: 4px;
  height: 20px;
  border-radius: 2px;
}

.timeline-track.imu .pulse {
  background: #3b82f6;
}

.timeline-track.lidar .pulse {
  background: #22c55e;
}

.timeline-track.camera .pulse {
  background: #a855f7;
}

.timeline-note {
  padding: 1rem;
  background: rgba(59, 130, 246, 0.1);
  border-left: 3px solid #3b82f6;
  color: #cbd5e1;
  font-size: 0.9rem;
  line-height: 1.6;
}

/* ANIMATIONS */
@keyframes pulse-sensor {
  0%,
  100% {
    transform: scale(1);
    opacity: 1;
  }
  50% {
    transform: scale(1.1);
    opacity: 0.8;
  }
}

@keyframes ring-expand {
  0% {
    transform: scale(1);
    opacity: 0.3;
  }
  50% {
    transform: scale(1.3);
    opacity: 0;
  }
  100% {
    transform: scale(1);
    opacity: 0.3;
  }
}

@keyframes ray-pulse {
  0% {
    opacity: 0;
  }
  50% {
    opacity: 1;
  }
  100% {
    opacity: 0;
  }
}

@media (max-width: 1024px) {
  .hero-section {
    flex-direction: column;
    padding: 3rem 1.5rem;
  }
  .noise-lab {
    grid-template-columns: 1fr;
  }
  .lab-controls {
    border-right: none;
    border-bottom: 1px solid #334155;
  }
}

/* LIDAR CONFIGURATOR */
.lidar-config-tool {
  background: #0f172a;
  border: 1px solid #334155;
  border-radius: 12px;
  overflow: hidden;
}

.config-grid {
  display: grid;
  grid-template-columns: 300px 1fr;
  min-height: 350px;
}

.config-controls {
  padding: 2rem;
  background: rgba(30, 41, 59, 0.5);
  border-right: 1px solid #334155;
  display: flex;
  flex-direction: column;
  gap: 2rem;
}

.config-viz {
  padding: 2rem;
  background: linear-gradient(180deg, #1e293b 0%, #0f172a 100%);
  display: flex;
  align-items: center;
  justify-content: center;
}

.lidar-scan-viz {
  width: 100%;
  max-width: 400px;
}

.scan-canvas {
  width: 100%;
  height: auto;
}

.viz-stats {
  text-align: center;
  margin-top: 1rem;
  font-family: monospace;
  color: #22c55e;
  font-size: 0.85rem;
}

/* CAMERA FRUSTUM */
.camera-frustum-viz {
  background: rgba(15, 23, 42, 0.6);
  padding: 2rem;
  border-radius: 16px;
  border: 1px solid rgba(148, 163, 184, 0.1);
}

.frustum-container {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 2rem;
  align-items: center;
}

.frustum-3d {
  perspective: 600px;
  display: flex;
  justify-content: center;
  align-items: center;
  height: 250px;
}

.frustum-pyramid {
  width: 100px;
  height: 150px;
  position: relative;
  transform-style: preserve-3d;
  transform: rotateX(-20deg) rotateY(30deg);
  animation: rotate-frustum 10s infinite linear;
}

.frustum-face {
  position: absolute;
  background: rgba(139, 92, 246, 0.2);
  border: 1px solid #a855f7;
}

.frustum-face.front {
  width: 100px;
  height: 150px;
  transform: translateZ(75px);
}

.frustum-face.back {
  width: 50px;
  height: 75px;
  top: 37.5px;
  left: 25px;
  transform: translateZ(-75px);
}

.frustum-params {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.param-row {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 0.75rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 8px;
}

.param-label {
  color: #94a3b8;
  font-weight: 600;
}

.param-row code {
  color: #a855f7;
  font-family: monospace;
  font-size: 1rem;
}

/* IMU COMPONENTS */
.imu-components-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
  gap: 1.5rem;
}

.imu-component {
  background: rgba(15, 23, 42, 0.6);
  border-radius: 12px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  align-items: center;
  text-align: center;
  gap: 1rem;
  border: 2px solid;
}

.imu-component.accel {
  border-color: #3b82f6;
}

.imu-component.gyro {
  border-color: #a855f7;
}

.imu-component.mag {
  border-color: #22c55e;
}

.comp-title {
  font-size: 1.2rem;
  font-weight: 700;
  color: #fff;
}

.comp-desc {
  color: #cbd5e1;
  font-size: 0.9rem;
  line-height: 1.6;
}

.comp-noise {
  padding: 0.5rem 1rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 6px;
  font-size: 0.85rem;
  color: #94a3b8;
}

.comp-noise code {
  color: #fbbf24;
}

/* DIFF DRIVE VIZ */
.diff-drive-viz {
  background: rgba(15, 23, 42, 0.6);
  padding: 2rem;
  border-radius: 16px;
  border: 1px solid rgba(148, 163, 184, 0.1);
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 2rem;
}

.drive-diagram {
  display: flex;
  align-items: center;
  justify-content: center;
}

.robot-top-view {
  width: 100%;
  max-width: 250px;
}

.drive-equations {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.eq-title {
  font-size: 1.2rem;
  font-weight: 700;
  color: #fff;
  margin-bottom: 0.5rem;
}

.eq-row {
  display: flex;
  flex-direction: column;
  gap: 0.25rem;
  padding: 0.75rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 6px;
}

.eq-row code {
  font-family: 'Fira Code', monospace;
  color: #60a5fa;
  font-size: 0.95rem;
}

.eq-row span {
  color: #94a3b8;
  font-size: 0.8rem;
}

.eq-params {
  margin-top: 1rem;
  padding: 1rem;
  background: rgba(59, 130, 246, 0.1);
  border-radius: 6px;
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.eq-params code {
  color: #fbbf24;
}

/* ANIMATIONS */
@keyframes rotate-frustum {
  from {
    transform: rotateX(-20deg) rotateY(0deg);
  }
  to {
    transform: rotateX(-20deg) rotateY(360deg);
  }
}

@media (max-width: 1024px) {
  .config-grid,
  .frustum-container,
  .diff-drive-viz {
    grid-template-columns: 1fr;
  }
}

/* SENSOR SUITE VIZ */
.sensor-suite-viz {
  background: #0f172a;
  border: 1px solid #334155;
  border-radius: 12px;
  overflow: hidden;
}

.suite-header {
  background: #1e293b;
  padding: 1.5rem;
  display: flex;
  align-items: center;
  gap: 1rem;
  border-bottom: 1px solid #334155;
}

.suite-title {
  font-size: 1.3rem;
  font-weight: 700;
  color: #fff;
}

.suite-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 1rem;
  padding: 2rem;
  background: linear-gradient(180deg, #1e293b 0%, #0f172a 100%);
}

.sensor-card {
  background: rgba(15, 23, 42, 0.6);
  border-radius: 12px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  align-items: center;
  text-align: center;
  gap: 0.75rem;
  border: 2px solid;
  transition: transform 0.2s;
}

.sensor-card:hover {
  transform: translateY(-5px);
}

.sensor-card.lidar-card {
  border-color: #22c55e;
}

.sensor-card.camera-card {
  border-color: #a855f7;
}

.sensor-card.imu-card {
  border-color: #3b82f6;
}

.sensor-card.encoder-card {
  border-color: #f59e0b;
}

.sensor-name {
  font-weight: 700;
  color: #fff;
  font-size: 1rem;
}

.sensor-spec {
  color: #94a3b8;
  font-size: 0.85rem;
}

.sensor-freq {
  background: rgba(0, 0, 0, 0.3);
  padding: 0.25rem 0.75rem;
  border-radius: 99px;
  font-family: monospace;
  color: #22d3ee;
  font-size: 0.8rem;
}

.suite-metrics {
  display: flex;
  justify-content: space-around;
  padding: 1.5rem;
  background: rgba(0, 0, 0, 0.3);
  border-top: 1px solid #334155;
}

.metric-item {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.5rem;
}

.metric-label {
  color: #64748b;
  font-size: 0.8rem;
  text-transform: uppercase;
  letter-spacing: 0.5px;
}

.metric-item code {
  color: #fbbf24;
  font-size: 1.1rem;
  font-weight: 700;
}

/* CHALLENGE CARD */
.challenge-card {
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid #d946ef;
  border-radius: 16px;
  padding: 2rem;
}

.challenge-header {
  display: flex;
  gap: 1.5rem;
  margin-bottom: 2rem;
}

.c-icon {
  font-size: 3rem;
  color: #d946ef;
}

.c-title {
  font-size: 1.25rem;
  font-weight: 700;
  color: #f0abfc;
  margin-bottom: 0.5rem;
}

.c-desc {
  color: #cbd5e1;
  font-size: 0.95rem;
}

.challenge-options {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.c-option {
  display: flex;
  gap: 1rem;
  padding: 1rem;
  background: rgba(0, 0, 0, 0.2);
  border-radius: 8px;
  cursor: default;
}

.c-option.correct {
  border: 1px solid #22c55e;
  background: rgba(34, 197, 94, 0.1);
}

.opt-radio {
  width: 30px;
  height: 30px;
  border-radius: 50%;
  border: 2px solid #64748b;
  display: flex;
  align-items: center;
  justify-content: center;
  font-weight: 700;
  color: #cbd5e1;
  flex-shrink: 0;
}

.c-option.correct .opt-radio {
  border-color: #22c55e;
  color: #22c55e;
}

.opt-text {
  color: #fff;
  flex: 1;
}

.opt-feedback {
  margin-top: 0.5rem;
  color: #86efac;
  font-size: 0.9rem;
}

/* VIDEO STYLES */
.video-container {
  background: #0f172a;
  border: 1px solid #334155;
  border-radius: 12px;
  overflow: hidden;
  padding: 1rem;
  box-shadow: 0 10px 30px -5px rgba(0, 0, 0, 0.5);
}

.video-wrapper {
  position: relative;
  padding-bottom: 56.25%;
  height: 0;
  overflow: hidden;
  border-radius: 8px;
  background: #000;
}

.video-wrapper iframe {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
}

.video-caption {
  padding: 1rem 0.5rem 0.5rem 0.5rem;
  color: #94a3b8;
  font-size: 0.9rem;
  display: flex;
  align-items: center;
  gap: 0.75rem;
}

/* SUMMARY & NEXT STEP */
.summary-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 1rem;
}

.summary-item {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 8px;
  padding: 1rem;
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.summary-item code {
  font-family: 'Fira Code', monospace;
  color: #22d3ee;
  font-size: 0.95rem;
  font-weight: 700;
}

.summary-item span {
  color: #cbd5e1;
  font-size: 0.85rem;
}

.next-step {
  background: rgba(59, 130, 246, 0.1);
  padding: 2rem;
  border-radius: 12px;
  display: inline-block;
  border: 1px solid #3b82f6;
  text-align: center;
  width: 100%;
}

.next-label {
  color: #94a3b8;
  font-size: 0.9rem;
  text-transform: uppercase;
  letter-spacing: 1px;
  margin-bottom: 0.5rem;
}

.next-title {
  color: #fff;
  font-size: 1.3rem;
  font-weight: 700;
  margin-bottom: 1rem;
}
</style>
