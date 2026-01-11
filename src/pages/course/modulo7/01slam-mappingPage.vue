<template>
  <LessonContainer>
    <!-- ========================================================================================
         HERO SECTION: SLAM - THE CHICKEN AND EGG PROBLEM
         Visual: Robot exploring + map building animation
         ======================================================================================== -->
    <div class="hero-section text-white q-mb-xl">
      <div class="hero-content">
        <div class="hero-badge"><q-icon name="explore" /> MÓDULO 7.1: SLAM MAPPING</div>
        <h1 class="hero-title">
          SLAM: <span class="gradient-text">Simultaneous Localization and Mapping</span>
        </h1>
        <p class="hero-subtitle">
          El problema del huevo y la gallina de la robótica móvil. Para construir un mapa necesitas
          saber dónde estás, pero para saber dónde estás necesitas un mapa. SLAM resuelve ambos
          problemas <strong>simultáneamente</strong> usando probabilidades y optimización.
        </p>

        <div class="hero-stats">
          <div class="stat-item">
            <div class="stat-val">2 Problemas</div>
            <div class="stat-label">Localization + Mapping</div>
          </div>
          <div class="stat-item">
            <div class="stat-val">1 Solución</div>
            <div class="stat-label">Bayesian Inference</div>
          </div>
          <div class="stat-item">
            <div class="stat-val">∞ Apps</div>
            <div class="stat-label">Autonomous Navigation</div>
          </div>
        </div>
      </div>

      <div class="hero-viz">
        <!-- SLAM Animation SVG -->
        <svg viewBox="0 0 300 300" class="slam-animation">
          <defs>
            <radialGradient id="robotGlow">
              <stop offset="0%" stop-color="#3b82f6" stop-opacity="1" />
              <stop offset="100%" stop-color="#3b82f6" stop-opacity="0" />
            </radialGradient>
            <radialGradient id="mapGlow">
              <stop offset="0%" stop-color="#22c55e" stop-opacity="0.8" />
              <stop offset="100%" stop-color="#22c55e" stop-opacity="0" />
            </radialGradient>
          </defs>

          <!-- Map Grid (building) -->
          <g class="map-grid">
            <rect
              x="50"
              y="50"
              width="200"
              height="200"
              fill="none"
              stroke="#22c55e"
              stroke-width="2"
              opacity="0.3"
              class="grid-outline"
            />
            <line
              x1="50"
              y1="100"
              x2="250"
              y2="100"
              stroke="#22c55e"
              stroke-width="1"
              opacity="0.2"
              class="grid-line g1"
            />
            <line
              x1="50"
              y1="150"
              x2="250"
              y2="150"
              stroke="#22c55e"
              stroke-width="1"
              opacity="0.2"
              class="grid-line g2"
            />
            <line
              x1="50"
              y1="200"
              x2="250"
              y2="200"
              stroke="#22c55e"
              stroke-width="1"
              opacity="0.2"
              class="grid-line g3"
            />
            <line
              x1="100"
              y1="50"
              x2="100"
              y2="250"
              stroke="#22c55e"
              stroke-width="1"
              opacity="0.2"
              class="grid-line g4"
            />
            <line
              x1="150"
              y1="50"
              x2="150"
              y2="250"
              stroke="#22c55e"
              stroke-width="1"
              opacity="0.2"
              class="grid-line g5"
            />
            <line
              x1="200"
              y1="50"
              x2="200"
              y2="250"
              stroke="#22c55e"
              stroke-width="1"
              opacity="0.2"
              class="grid-line g6"
            />
          </g>

          <!-- Robot (moving) -->
          <g class="robot" transform="translate(150, 150)">
            <circle cx="0" cy="0" r="30" fill="url(#robotGlow)" opacity="0.3" class="robot-glow" />
            <circle cx="0" cy="0" r="15" fill="#3b82f6" stroke="#60a5fa" stroke-width="2" />
            <path d="M -10,-5 L 10,0 L -10,5 Z" fill="#fff" />
          </g>

          <!-- Uncertainty Cloud -->
          <circle cx="150" cy="150" r="50" fill="url(#mapGlow)" opacity="0.2" class="uncertainty" />

          <!-- Trajectory -->
          <path
            d="M 80,80 Q 120,100 150,150"
            stroke="#fbbf24"
            stroke-width="2"
            fill="none"
            stroke-dasharray="5,5"
            class="trajectory"
          />
        </svg>
      </div>
    </div>

    <!-- ========================================================================================
         BLOCK A: MATHEMATICAL FOUNDATIONS
         Depth: Bayesian Inference, Particle Filters, Graph Optimization
         ======================================================================================== -->
    <div class="content-block">
      <SectionTitle>1. Fundamentos Matemáticos: El Problema Probabilístico</SectionTitle>

      <TextBlock>
        SLAM es un problema de <strong>inferencia bayesiana</strong>. Queremos estimar la
        distribución de probabilidad conjunta de la trayectoria del robot y el mapa del entorno,
        dadas todas las mediciones de sensores y comandos de control.
      </TextBlock>

      <AlertBlock type="warning" title="El Dilema Fundamental">
        <strong>Localización:</strong> "¿Dónde estoy en el mapa?" → Requiere un mapa conocido.
        <br />
        <strong>Mapping:</strong> "¿Cómo es el entorno?" → Requiere conocer tu posición.
        <br /><br />
        SLAM rompe este círculo vicioso usando <strong>probabilidades</strong> y
        <strong>optimización</strong>. No busca una respuesta exacta, sino la distribución de
        probabilidad más probable.
      </AlertBlock>

      <!-- BAYES EQUATION -->
      <div class="math-foundation q-my-lg">
        <div class="foundation-card">
          <div class="foundation-header">
            <q-icon name="calculate" size="md" />
            <span>Ecuación de Bayes para SLAM</span>
          </div>
          <div class="foundation-body">
            <div class="equation-block">
              <div class="eq-title">Distribución Posterior (Lo que queremos):</div>
              <div class="eq-formula">$$p(x_{0:t}, m \mid z_{1:t}, u_{0:t})$$</div>
              <div class="eq-desc">
                <strong>Donde:</strong>
                <ul>
                  <li>$x_{0:t}$ = Trayectoria completa del robot (poses desde tiempo 0 hasta t)</li>
                  <li>$m$ = Mapa del entorno (occupancy grid, landmarks, etc.)</li>
                  <li>$z_{1:t}$ = Todas las mediciones de sensores (lidar, cámara, IMU)</li>
                  <li>$u_{0:t}$ = Todos los comandos de control (velocidades, aceleraciones)</li>
                </ul>
              </div>
            </div>

            <div class="equation-block q-mt-md">
              <div class="eq-title">Descomposición Recursiva (Filtro de Bayes):</div>
              <div class="eq-formula">
                $$p(x_t, m \mid z_{1:t}, u_{0:t}) = \eta \cdot p(z_t \mid x_t, m) \int p(x_t \mid
                x_{t-1}, u_t) \cdot p(x_{t-1}, m \mid z_{1:t-1}, u_{0:t-1}) \, dx_{t-1}$$
              </div>
              <div class="eq-desc">
                <strong>Tres componentes críticos:</strong>
                <ul>
                  <li>
                    <strong>Modelo de Medición</strong> $p(z_t \mid x_t, m)$: "¿Qué tan probable es
                    esta medición si estoy en $x_t$ con mapa $m$?"
                  </li>
                  <li>
                    <strong>Modelo de Movimiento</strong> $p(x_t \mid x_{t-1}, u_t)$: "¿Dónde
                    terminaré si ejecuto comando $u_t$ desde $x_{t-1}$?"
                  </li>
                  <li>
                    <strong>Creencia Anterior</strong> $p(x_{t-1}, m \mid z_{1:t-1}, u_{0:t-1})$:
                    "¿Qué sabía antes de esta medición?"
                  </li>
                </ul>
              </div>
            </div>
          </div>
        </div>
      </div>

      <AlertBlock type="info" title="¿Por qué es tan difícil?">
        El espacio de estados es <strong>infinito</strong>. Para un mapa de 100x100 celdas y 1000
        poses, tienes que estimar ~13,000 variables simultáneamente. Resolver esto exactamente es
        computacionalmente intratable ($O(n^3)$ para n variables). <br /><br />
        <strong>Soluciones:</strong>
        <ul>
          <li>
            <strong>Filtros (Online):</strong> Particle Filter, EKF-SLAM → Mantienen solo la
            creencia actual, descartan historia
          </li>
          <li>
            <strong>Smoothing (Batch/Online):</strong> Graph-SLAM, Bundle Adjustment → Optimizan
            toda la trayectoria simultáneamente
          </li>
        </ul>
      </AlertBlock>
    </div>

    <!-- =====================================================================================
         BLOQUE B: FILTRO DE PARTÍCULAS (PARTICLE FILTER / MONTE CARLO LOCALIZATION)
         ===================================================================================== -->
    <div class="section-group">
      <SectionTitle>2. Particle Filter SLAM (FastSLAM)</SectionTitle>

      <TextBlock>
        El <strong>Filtro de Partículas</strong> (Particle Filter) es una técnica de Monte Carlo que
        representa la distribución de probabilidad posterior mediante un conjunto de muestras
        ponderadas (partículas). En SLAM, cada partícula mantiene una hipótesis completa del estado
        del robot y el mapa.
      </TextBlock>

      <AlertBlock type="info" title="FastSLAM: Factorización Clave">
        FastSLAM explota la estructura condicional del problema SLAM:
        <br /><br />
        <div class="formula">
          $p(x_{1:t}, m | z_{1:t}, u_{1:t}) = p(x_{1:t} | z_{1:t}, u_{1:t}) \prod_{i=1}^{M} p(m_i |
          x_{1:t}, z_{1:t})$
        </div>
        <br />
        <strong>Interpretación:</strong> Si conoces la trayectoria del robot ($x_{1:t}$), los
        landmarks del mapa ($m_i$) son <em>condicionalmente independientes</em>. Esto permite usar
        un <strong>Rao-Blackwellized Particle Filter</strong>:
        <ul>
          <li>
            <strong>Partículas</strong> → Representan trayectorias del robot $x_{1:t}$ (pose path)
          </li>
          <li>
            <strong>Filtros de Kalman</strong> → Cada partícula mantiene M filtros EKF para estimar
            landmarks $m_i$
          </li>
        </ul>
        <strong>Ventaja:</strong> Complejidad $O(M \log N)$ en lugar de $O(M^3)$ del EKF-SLAM
        completo.
      </AlertBlock>

      <div class="math-foundation-grid">
        <!-- Algoritmo FastSLAM 1.0 -->
        <div class="math-eq-card">
          <div class="eq-header">
            <q-icon name="psychology" size="2rem" color="purple-4" />
            <span>Algoritmo FastSLAM 1.0</span>
          </div>
          <div class="eq-formula">
            $\text{Para cada partícula } k = 1 \dots N:$
            <br />
            $1. \text{ Muestreo: } x_t^{[k]} \sim p(x_t | x_{t-1}^{[k]}, u_t)$
            <br />
            $2. \text{ Peso: } w_t^{[k]} = p(z_t | x_t^{[k]}, m_{t-1}^{[k]})$
            <br />
            $3. \text{ Actualizar landmarks con EKF}$
            <br />
            $4. \text{ Resamplear si } N_{eff} &lt; N_{threshold}$
          </div>
          <div class="eq-desc">
            <strong>Paso 1 (Predicción):</strong> Propagar cada partícula usando el modelo de
            movimiento (odometría + ruido gaussiano). <br /><br />
            <strong>Paso 2 (Corrección):</strong> Calcular peso comparando la observación real $z_t$
            con la predicción del mapa de la partícula. <br /><br />
            <strong>Paso 3 (Mapping):</strong> Para cada landmark observado, actualizar su
            estimación con un EKF individual (media $\mu_i$ y covarianza $\Sigma_i$). <br /><br />
            <strong>Paso 4 (Resampling):</strong> Eliminar partículas con bajo peso y duplicar las
            de alto peso para evitar degeneración.
          </div>
        </div>

        <!-- Modelo de Movimiento -->
        <div class="math-eq-card">
          <div class="eq-header">
            <q-icon name="directions_run" size="2rem" color="cyan-4" />
            <span>Modelo de Movimiento (Odometría)</span>
          </div>
          <div class="eq-formula">
            $x_t = x_{t-1} + \begin{bmatrix} \Delta x \\ \Delta y \\ \Delta \theta \end{bmatrix} +
            \mathcal{N}(0, R_t)$
          </div>
          <div class="eq-desc">
            Donde:
            <ul>
              <li>
                <strong>$\Delta x, \Delta y, \Delta \theta$:</strong> Cambio en pose según odometría
                (encoders de ruedas)
              </li>
              <li>
                <strong>$R_t$:</strong> Matriz de covarianza del ruido (depende de la superficie,
                velocidad, etc.)
              </li>
              <li>
                <strong>$\mathcal{N}(0, R_t)$:</strong> Ruido gaussiano que modela deslizamiento,
                derrape, imprecisión de encoders
              </li>
            </ul>
            <strong>Implementación:</strong> En ROS 2, esto se obtiene del topic
            <code>/odom</code> (tipo <code>nav_msgs/Odometry</code>). El ruido se configura con
            parámetros <code>alpha1..alpha4</code> en AMCL o SLAM Toolbox.
          </div>
        </div>

        <!-- Modelo de Observación -->
        <div class="math-eq-card">
          <div class="eq-header">
            <q-icon name="sensors" size="2rem" color="orange-4" />
            <span>Modelo de Observación (Lidar)</span>
          </div>
          <div class="eq-formula">
            $p(z_t | x_t, m) = \prod_{i=1}^{K} p(z_t^i | x_t, m)$
            <br />
            $p(z_t^i | x_t, m) = z_{hit} \cdot \mathcal{N}(\hat{z}_t^i, \sigma_{hit}^2) + z_{rand}
            \cdot \text{Uniform}(0, z_{max})$
          </div>
          <div class="eq-desc">
            <strong>Modelo de Mezcla:</strong> Cada rayo láser $z_t^i$ se modela como combinación
            de:
            <ul>
              <li>
                <strong>$z_{hit}$:</strong> Probabilidad de medición correcta (gaussiana centrada en
                $\hat{z}_t^i$, la distancia esperada según el mapa)
              </li>
              <li>
                <strong>$z_{rand}$:</strong> Probabilidad de medición aleatoria (ruido, reflexiones,
                objetos dinámicos)
              </li>
              <li>
                <strong>Otros términos:</strong> $z_{short}$ (obstáculos no mapeados), $z_{max}$
                (fallo de medición)
              </li>
            </ul>
            <strong>Cálculo de $\hat{z}_t^i$:</strong> Ray-casting en el mapa $m$ desde la pose
            $x_t$ en la dirección del rayo $i$.
          </div>
        </div>

        <!-- Resampling -->
        <div class="math-eq-card">
          <div class="eq-header">
            <q-icon name="autorenew" size="2rem" color="green-4" />
            <span>Resampling (Low Variance Sampler)</span>
          </div>
          <div class="eq-formula">
            $r \sim \text{Uniform}[0, 1/N]$
            <br />
            $c = w_1, \quad i = 1$
            <br />
            $\text{Para } j = 1 \dots N:$
            <br />
            $\quad u = r + (j-1)/N$
            <br />
            $\quad \text{Mientras } u > c: \quad i = i+1, \quad c = c + w_i$
            <br />
            $\quad \text{Agregar } x_i \text{ a nuevo conjunto}$
          </div>
          <div class="eq-desc">
            <strong>Objetivo:</strong> Evitar degeneración del filtro (todas las partículas con peso
            ~0 excepto una). <br /><br />
            <strong>Low Variance Sampler:</strong> Algoritmo determinístico que garantiza que
            partículas con peso $w_i$ sean seleccionadas aproximadamente $N \cdot w_i$ veces.
            <br /><br />
            <strong>Criterio de Resampling:</strong> Solo resamplear cuando el número efectivo de
            partículas cae por debajo de un umbral:
            <br />
            <div class="formula">$N_{eff} = 1 / \sum_{i=1}^{N} (w_i)^2 &lt; N_{threshold}$</div>
          </div>
        </div>
      </div>

      <AlertBlock type="warning" title="Problema de Degeneración">
        En FastSLAM, si el modelo de movimiento es muy ruidoso o el modelo de observación muy
        preciso, las partículas pueden <strong>colapsar</strong> a una sola hipótesis
        prematuramente. <br /><br />
        <strong>Soluciones:</strong>
        <ul>
          <li><strong>Adaptive Resampling:</strong> Solo resamplear cuando $N_{eff}$ es bajo</li>
          <li>
            <strong>Proposal Distribution:</strong> FastSLAM 2.0 usa $p(x_t | x_{t-1}, u_t, z_t)$ en
            lugar de $p(x_t | x_{t-1}, u_t)$ para incorporar la observación en el muestreo
          </li>
          <li>
            <strong>Aumentar N:</strong> Más partículas = mejor cobertura del espacio de estados
            (pero mayor costo computacional)
          </li>
        </ul>
      </AlertBlock>
    </div>

    <!-- =====================================================================================
         BLOQUE C: GRAPH-BASED SLAM (POSE GRAPH OPTIMIZATION)
         ===================================================================================== -->
    <div class="section-group">
      <SectionTitle>3. Graph-Based SLAM (Optimización de Pose Graph)</SectionTitle>

      <TextBlock>
        A diferencia de los filtros (que mantienen solo la creencia actual),
        <strong>Graph-SLAM</strong>
        construye un grafo donde:
        <ul>
          <li><strong>Nodos</strong> = Poses del robot $x_1, x_2, \dots, x_T$</li>
          <li>
            <strong>Aristas</strong> = Restricciones espaciales (odometría, loop closures,
            landmarks)
          </li>
        </ul>
        Luego resuelve un problema de <strong>optimización no lineal</strong> para encontrar la
        configuración de poses que mejor satisface todas las restricciones.
      </TextBlock>

      <div class="math-foundation-grid">
        <!-- Formulación del Problema -->
        <div class="math-eq-card">
          <div class="eq-header">
            <q-icon name="account_tree" size="2rem" color="indigo-4" />
            <span>Formulación: Mínimos Cuadrados No Lineales</span>
          </div>
          <div class="eq-formula">
            $x^* = \arg\min_{x} \sum_{(i,j) \in \mathcal{E}} e_{ij}(x_i, x_j)^T \Omega_{ij}
            e_{ij}(x_i, x_j)$
          </div>
          <div class="eq-desc">
            Donde:
            <ul>
              <li>
                <strong>$\mathcal{E}$:</strong> Conjunto de aristas (restricciones) en el grafo
              </li>
              <li>
                <strong>$e_{ij}(x_i, x_j)$:</strong> Función de error que mide la discrepancia entre
                la pose relativa observada $z_{ij}$ y la predicha $\hat{z}_{ij}(x_i, x_j)$
              </li>
              <li>
                <strong>$\Omega_{ij}$:</strong> Matriz de información (inversa de la covarianza) que
                pondera la confianza en la medición
              </li>
            </ul>
            <strong>Ejemplo de error:</strong> Para odometría 2D:
            <br />
            <div class="formula">
              $e_{ij} = \begin{bmatrix} \Delta x \\ \Delta y \\ \Delta \theta \end{bmatrix}_{obs} -
              R(\theta_i)^T (t_j - t_i)$
            </div>
          </div>
        </div>

        <!-- Gauss-Newton -->
        <div class="math-eq-card">
          <div class="eq-header">
            <q-icon name="functions" size="2rem" color="pink-4" />
            <span>Solución: Gauss-Newton / Levenberg-Marquardt</span>
          </div>
          <div class="eq-formula">
            $H \Delta x = -b$
            <br />
            $H = J^T \Omega J, \quad b = J^T \Omega e$
          </div>
          <div class="eq-desc">
            <strong>Algoritmo iterativo:</strong>
            <ul>
              <li>
                <strong>1. Linealizar:</strong> Calcular Jacobiano $J$ de $e(x)$ alrededor de la
                estimación actual $\bar{x}$
              </li>
              <li>
                <strong>2. Resolver:</strong> Sistema lineal $H \Delta x = -b$ (matriz $H$ es
                <em>sparse</em> gracias a la estructura del grafo)
              </li>
              <li><strong>3. Actualizar:</strong> $\bar{x} \leftarrow \bar{x} + \Delta x$</li>
              <li><strong>4. Repetir</strong> hasta convergencia</li>
            </ul>
            <strong>Librerías:</strong> g2o, GTSAM, Ceres Solver (usadas por SLAM Toolbox y
            Cartographer).
          </div>
        </div>

        <!-- Loop Closure -->
        <div class="math-eq-card">
          <div class="eq-header">
            <q-icon name="loop" size="2rem" color="teal-4" />
            <span>Loop Closure Detection</span>
          </div>
          <div class="eq-formula">
            $\text{Similarity}(I_i, I_j) > \tau \implies \text{Add edge } (i, j)$
          </div>
          <div class="eq-desc">
            <strong>Problema:</strong> Detectar cuándo el robot regresa a un lugar previamente
            visitado. <br /><br />
            <strong>Técnicas:</strong>
            <ul>
              <li>
                <strong>Bag of Words (BoW):</strong> Representar scans láser o imágenes como
                vectores de características (SIFT, ORB) y comparar con base de datos (DBoW2, DBoW3)
              </li>
              <li>
                <strong>Scan Matching:</strong> Alinear scan actual con scans históricos usando ICP
                (Iterative Closest Point) o correlación
              </li>
              <li>
                <strong>Geometric Verification:</strong> Verificar que la transformación estimada
                $T_{ij}$ sea consistente con la geometría
              </li>
            </ul>
            <strong>Impacto:</strong> Un loop closure correcto <em>corrige drift acumulado</em> en
            toda la trayectoria. Un falso positivo puede <strong>destruir</strong> el mapa.
          </div>
        </div>

        <!-- Robust Kernels -->
        <div class="math-eq-card">
          <div class="eq-header">
            <q-icon name="shield" size="2rem" color="red-4" />
            <span>Robust Kernels (Outlier Rejection)</span>
          </div>
          <div class="eq-formula">
            $\rho(e) = \begin{cases} e^2 & \text{if } |e| \leq \delta \\ 2\delta|e| - \delta^2 &
            \text{otherwise} \end{cases}$
          </div>
          <div class="eq-desc">
            <strong>Problema:</strong> Loop closures incorrectos (outliers) pueden arruinar la
            optimización. <br /><br />
            <strong>Solución:</strong> Usar funciones de costo robustas (Huber, Cauchy, Tukey) que
            reducen el peso de errores grandes:
            <ul>
              <li><strong>Huber:</strong> Cuadrático para errores pequeños, lineal para grandes</li>
              <li>
                <strong>Cauchy:</strong> $\rho(e) = \log(1 + e^2)$ → Rechaza outliers más
                agresivamente
              </li>
              <li>
                <strong>Dynamic Covariance Scaling (DCS):</strong> Ajustar $\Omega_{ij}$
                dinámicamente según residuos
              </li>
            </ul>
          </div>
        </div>
      </div>

      <AlertBlock type="info" title="Graph-SLAM vs Filtros: ¿Cuándo usar cada uno?">
        <strong>Filtros (Particle Filter, EKF):</strong>
        <ul>
          <li>✅ <strong>Online:</strong> Actualizan en tiempo real con cada medición</li>
          <li>✅ <strong>Memoria constante:</strong> Solo almacenan estado actual</li>
          <li>❌ <strong>No corrigen pasado:</strong> Drift acumulado no se elimina</li>
          <li>❌ <strong>Aproximaciones:</strong> Linearización (EKF) o muestreo (PF)</li>
        </ul>
        <br />
        <strong>Graph-SLAM:</strong>
        <ul>
          <li>
            ✅ <strong>Globalmente consistente:</strong> Loop closures corrigen toda la trayectoria
          </li>
          <li>✅ <strong>Óptimo:</strong> Solución de máxima verosimilitud (si converge)</li>
          <li>❌ <strong>Memoria creciente:</strong> Almacena todas las poses y landmarks</li>
          <li>
            ❌ <strong>Costo computacional:</strong> Optimización puede ser lenta para grafos
            grandes
          </li>
        </ul>
        <br />
        <strong>Híbridos (SLAM Toolbox, Cartographer):</strong> Usan filtros para estimación online
        + graph optimization en background para corrección global.
      </AlertBlock>
    </div>

    <!-- =====================================================================================
         BLOQUE D: SENSORES PARA SLAM
         ===================================================================================== -->
    <div class="section-group">
      <SectionTitle>4. Sensores para SLAM: Lidar, Cámaras y Profundidad</SectionTitle>

      <TextBlock>
        La elección del sensor determina el tipo de SLAM que puedes ejecutar, la precisión del mapa,
        y los requisitos computacionales. Aquí comparamos las tres familias principales:
      </TextBlock>

      <div class="sensor-comparison-grid">
        <!-- Lidar 2D -->
        <div class="sensor-card lidar-2d">
          <div class="sensor-header">
            <q-icon name="radar" size="3rem" />
            <h3>Lidar 2D</h3>
            <div class="sensor-badge">Más Común en ROS 2</div>
          </div>
          <div class="sensor-body">
            <div class="sensor-spec">
              <strong>Principio:</strong> Mide distancia con láser rotativo (Time-of-Flight)
            </div>
            <div class="sensor-spec">
              <strong>Salida:</strong> Array de distancias en un plano (360° × 1 línea)
            </div>
            <div class="sensor-spec">
              <strong>Rango:</strong> 0.1m - 30m (típico), hasta 100m (industrial)
            </div>
            <div class="sensor-spec"><strong>Frecuencia:</strong> 5-40 Hz</div>
            <div class="sensor-spec"><strong>Precisión:</strong> ±2-5 cm</div>
            <div class="sensor-pros">
              <strong>Ventajas:</strong>
              <ul>
                <li>✅ Muy preciso en interiores estructurados</li>
                <li>✅ No afectado por iluminación</li>
                <li>✅ Algoritmos maduros (SLAM Toolbox, Cartographer)</li>
                <li>✅ Bajo costo computacional</li>
              </ul>
            </div>
            <div class="sensor-cons">
              <strong>Desventajas:</strong>
              <ul>
                <li>❌ Solo 2D (no detecta obstáculos arriba/abajo)</li>
                <li>❌ Problemas con vidrio, superficies especulares</li>
                <li>❌ Costo: $100 (RPLidar A1) - $5000+ (SICK, Hokuyo)</li>
              </ul>
            </div>
            <div class="sensor-examples">
              <strong>Ejemplos:</strong> RPLidar A1/A2/A3, Hokuyo UTM-30LX, SICK TiM series
            </div>
          </div>
        </div>

        <!-- Lidar 3D -->
        <div class="sensor-card lidar-3d">
          <div class="sensor-header">
            <q-icon name="3d_rotation" size="3rem" />
            <h3>Lidar 3D</h3>
            <div class="sensor-badge">Alta Precisión</div>
          </div>
          <div class="sensor-body">
            <div class="sensor-spec">
              <strong>Principio:</strong> Múltiples láseres en diferentes ángulos verticales
            </div>
            <div class="sensor-spec"><strong>Salida:</strong> Nube de puntos 3D (Point Cloud)</div>
            <div class="sensor-spec"><strong>Rango:</strong> 0.5m - 200m (Velodyne, Ouster)</div>
            <div class="sensor-spec"><strong>Frecuencia:</strong> 10-20 Hz</div>
            <div class="sensor-spec"><strong>Precisión:</strong> ±2-3 cm</div>
            <div class="sensor-pros">
              <strong>Ventajas:</strong>
              <ul>
                <li>✅ Mapa 3D completo del entorno</li>
                <li>✅ Excelente para exteriores y terrenos irregulares</li>
                <li>✅ Usado en vehículos autónomos (Waymo, Cruise)</li>
              </ul>
            </div>
            <div class="sensor-cons">
              <strong>Desventajas:</strong>
              <ul>
                <li>❌ Muy costoso: $4,000 - $75,000+</li>
                <li>❌ Alto costo computacional (millones de puntos/seg)</li>
                <li>❌ Requiere algoritmos especializados (LOAM, LeGO-LOAM)</li>
              </ul>
            </div>
            <div class="sensor-examples">
              <strong>Ejemplos:</strong> Velodyne VLP-16 (Puck), Ouster OS1-64, Livox Mid-360
            </div>
          </div>
        </div>

        <!-- Cámara RGB-D -->
        <div class="sensor-card rgbd">
          <div class="sensor-header">
            <q-icon name="videocam" size="3rem" />
            <h3>Cámara RGB-D</h3>
            <div class="sensor-badge">Bajo Costo</div>
          </div>
          <div class="sensor-body">
            <div class="sensor-spec">
              <strong>Principio:</strong> Cámara RGB + sensor de profundidad (IR structured light o
              ToF)
            </div>
            <div class="sensor-spec">
              <strong>Salida:</strong> Imagen RGB + mapa de profundidad (640×480 - 1920×1080)
            </div>
            <div class="sensor-spec"><strong>Rango:</strong> 0.3m - 10m (típico)</div>
            <div class="sensor-spec"><strong>Frecuencia:</strong> 30-90 Hz</div>
            <div class="sensor-spec">
              <strong>Precisión:</strong> ±1-5 cm (depende de distancia)
            </div>
            <div class="sensor-pros">
              <strong>Ventajas:</strong>
              <ul>
                <li>✅ Muy económico: $100-$400</li>
                <li>✅ Información de color (útil para reconocimiento de objetos)</li>
                <li>✅ Bueno para interiores pequeños</li>
              </ul>
            </div>
            <div class="sensor-cons">
              <strong>Desventajas:</strong>
              <ul>
                <li>❌ Rango limitado (&lt; 10m)</li>
                <li>❌ No funciona en exteriores (luz solar interfiere con IR)</li>
                <li>❌ Problemas con superficies negras, reflectantes</li>
                <li>❌ Drift mayor que lidar en SLAM</li>
              </ul>
            </div>
            <div class="sensor-examples">
              <strong>Ejemplos:</strong> Intel RealSense D435i, Microsoft Kinect, Orbbec Astra
            </div>
          </div>
        </div>

        <!-- Cámara Monocular -->
        <div class="sensor-card monocular">
          <div class="sensor-header">
            <q-icon name="camera_alt" size="3rem" />
            <h3>Cámara Monocular</h3>
            <div class="sensor-badge">Visual SLAM</div>
          </div>
          <div class="sensor-body">
            <div class="sensor-spec">
              <strong>Principio:</strong> Estima profundidad por movimiento (Structure from Motion)
            </div>
            <div class="sensor-spec"><strong>Salida:</strong> Secuencia de imágenes RGB</div>
            <div class="sensor-spec">
              <strong>Rango:</strong> Depende de features visibles (1m - 100m+)
            </div>
            <div class="sensor-spec"><strong>Frecuencia:</strong> 30-120 Hz</div>
            <div class="sensor-spec">
              <strong>Precisión:</strong> Escala ambigua (requiere inicialización)
            </div>
            <div class="sensor-pros">
              <strong>Ventajas:</strong>
              <ul>
                <li>✅ Extremadamente económico ($10-$50)</li>
                <li>✅ Ligero y compacto</li>
                <li>✅ Usado en drones (ORB-SLAM, VINS-Mono)</li>
              </ul>
            </div>
            <div class="sensor-cons">
              <strong>Desventajas:</strong>
              <ul>
                <li>❌ No mide escala absoluta (mapa puede estar en unidades arbitrarias)</li>
                <li>❌ Falla en entornos sin textura (paredes blancas)</li>
                <li>❌ Muy sensible a motion blur, iluminación</li>
                <li>❌ Alto costo computacional (feature extraction, matching)</li>
              </ul>
            </div>
            <div class="sensor-examples">
              <strong>Ejemplos:</strong> Cualquier webcam, Logitech C920, Raspberry Pi Camera v2
            </div>
          </div>
        </div>
      </div>

      <AlertBlock type="info" title="Recomendación para Robots Móviles en Interiores">
        Para un robot móvil educativo/investigación en interiores:
        <br /><br />
        <strong>1. Lidar 2D (RPLidar A1/A2)</strong> → Mejor relación costo/beneficio para SLAM 2D
        <br />
        <strong>2. RGB-D (RealSense D435i)</strong> → Si necesitas detección de objetos + SLAM 3D
        <br />
        <strong>3. Lidar 3D (Livox Mid-360)</strong> → Si el presupuesto lo permite y necesitas
        mapas 3D de alta calidad <br /><br />
        <strong>Evitar:</strong> Cámara monocular sola para SLAM (a menos que sea un drone con
        restricciones de peso extremas).
      </AlertBlock>
    </div>

    <!-- =====================================================================================
         BLOQUE E: SLAM TOOLBOX (ROS 2 IMPLEMENTATION)
         ===================================================================================== -->
    <div class="section-group">
      <SectionTitle>5. SLAM Toolbox: Implementación Profesional en ROS 2</SectionTitle>

      <TextBlock>
        <strong>SLAM Toolbox</strong> es el estándar de facto para SLAM 2D en ROS 2. Desarrollado
        por Steve Macenski (autor de Nav2), combina:
        <ul>
          <li>
            <strong>Karto SLAM</strong> (graph-based SLAM con scan matching) como backend de mapping
          </li>
          <li>
            <strong>Pose Graph Optimization</strong> con Ceres Solver para loop closure correction
          </li>
          <li>
            <strong>Lifelong Mapping</strong> → Capacidad de actualizar mapas existentes (no solo
            crear nuevos)
          </li>
          <li>
            <strong>Serialización</strong> → Guardar/cargar mapas y pose graphs en formato
            .posegraph
          </li>
        </ul>
      </TextBlock>

      <AlertBlock type="info" title="Modos de Operación">
        SLAM Toolbox ofrece 3 modos principales:
        <br /><br />
        <strong>1. Mapping Mode (slam_toolbox::SynchronousSlamToolbox):</strong> Crea un mapa nuevo
        desde cero. Usa este modo para exploración inicial. <br /><br />
        <strong>2. Localization Mode (slam_toolbox::LocalizationSlamToolbox):</strong> Localiza el
        robot en un mapa existente (similar a AMCL pero con corrección de pose graph). <br /><br />
        <strong>3. Lifelong Mapping Mode:</strong> Actualiza un mapa existente con nuevas
        observaciones (útil para entornos dinámicos).
      </AlertBlock>

      <div class="code-section">
        <h4>Instalación y Configuración Básica</h4>

        <div class="code-block-container">
          <div class="code-header">
            <q-icon name="terminal" />
            <span>Instalación en ROS 2 Humble</span>
          </div>
          <pre><code class="language-bash"># Instalar SLAM Toolbox
sudo apt install ros-humble-slam-toolbox

# Verificar instalación
ros2 pkg list | grep slam_toolbox

# Crear workspace para tu proyecto
mkdir -p ~/slam_ws/src
cd ~/slam_ws/src
git clone https://github.com/SteveMacenski/slam_toolbox.git -b humble
cd ~/slam_ws
colcon build --packages-select slam_toolbox
source install/setup.bash</code></pre>
        </div>

        <div class="code-block-container">
          <div class="code-header">
            <q-icon name="settings" />
            <span>Archivo de Configuración: mapper_params_online_async.yaml</span>
          </div>
          <pre><code class="language-yaml">slam_toolbox:
  ros__parameters:
    # ==================== SOLVER PARAMETERS ====================
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # ==================== ODOMETRY PARAMETERS ====================
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    use_map_saver: true
    mode: mapping  # mapping | localization | lifelong

    # ==================== SCAN MATCHING ====================
    # Resolución del mapa (metros/celda)
    resolution: 0.05

    # Rango máximo del láser a considerar (metros)
    max_laser_range: 20.0

    # Umbral de distancia mínima para agregar nuevo nodo al grafo
    minimum_travel_distance: 0.5
    minimum_travel_heading: 0.5  # radianes

    # Parámetros de Scan Matcher (correlación)
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0

    # ==================== CORRELATION SEARCH SPACE ====================
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    # ==================== LOOP CLOSURE ====================
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # ==================== SCAN MATCHER ====================
    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0

    fine_search_angle_offset: 0.00349  # ~0.2 grados
    coarse_search_angle_offset: 0.349  # ~20 grados
    coarse_angle_resolution: 0.0349    # ~2 grados
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true

    # ==================== OPTIMIZATION ====================
    # Frecuencia de optimización del pose graph
    transform_publish_period: 0.02  # 50 Hz
    map_update_interval: 5.0        # segundos

    # Habilitar visualización en RViz
    enable_interactive_mode: true

    # ==================== PERFORMANCE ====================
    # Número de threads para optimización
    number_of_threads: 4

    # Tamaño del stack para procesamiento
    stack_size_to_use: 40000000  # 40 MB

    # ==================== DEBUG ====================
    debug_logging: false
    throttle_scans: 1  # Procesar 1 de cada N scans (1 = todos)</code></pre>
        </div>

        <div class="code-block-container">
          <div class="code-header">
            <q-icon name="rocket_launch" />
            <span>Launch File: online_async_launch.py</span>
          </div>
          <pre><code class="language-python">from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    config_file = os.path.join(slam_toolbox_dir, 'config', 'mapper_params_online_async.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=config_file,
        description='Full path to SLAM Toolbox parameters file'
    )

    # SLAM Toolbox Node
    start_async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', '/scan'),  # Ajustar según tu robot
        ]
    )

    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)

    # Add nodes
    ld.add_action(start_async_slam_toolbox_node)

    return ld</code></pre>
        </div>

        <div class="code-block-container">
          <div class="code-header">
            <q-icon name="play_arrow" />
            <span>Ejecución del Sistema SLAM</span>
          </div>
          <pre><code class="language-bash"># Terminal 1: Lanzar robot (simulado o real)
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Lanzar SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

# Terminal 3: Lanzar RViz para visualización
ros2 launch slam_toolbox rviz.launch.py

# Terminal 4: Teleoperar el robot para explorar
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Guardar el mapa cuando termines la exploración
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map</code></pre>
        </div>
      </div>

      <AlertBlock type="warning" title="Tuning de Parámetros Críticos">
        <strong>Para entornos pequeños (oficina, laboratorio):</strong>
        <ul>
          <li><code>minimum_travel_distance: 0.3</code> → Más nodos = mejor precisión</li>
          <li><code>loop_search_maximum_distance: 2.0</code> → Búsqueda de loops más agresiva</li>
          <li><code>resolution: 0.05</code> → Mapa detallado</li>
        </ul>
        <br />
        <strong>Para entornos grandes (almacén, exterior):</strong>
        <ul>
          <li>
            <code>minimum_travel_distance: 1.0</code> → Menos nodos = menor costo computacional
          </li>
          <li><code>loop_search_maximum_distance: 5.0</code> → Loops en áreas grandes</li>
          <li><code>resolution: 0.10</code> → Mapa menos detallado pero más rápido</li>
        </ul>
        <br />
        <strong>Si el mapa tiene "fantasmas" o artefactos:</strong>
        <ul>
          <li>Aumentar <code>link_match_minimum_response_fine</code> (más estricto en matching)</li>
          <li>Reducir <code>max_laser_range</code> (ignorar lecturas lejanas ruidosas)</li>
          <li>Aumentar <code>loop_match_minimum_response_fine</code> (evitar falsos loops)</li>
        </ul>
      </AlertBlock>
    </div>

    <!-- =====================================================================================
         BLOQUE F: CARTOGRAPHER (GOOGLE'S SLAM)
         ===================================================================================== -->
    <div class="section-group">
      <SectionTitle>6. Cartographer: SLAM de Google para 2D y 3D</SectionTitle>

      <TextBlock>
        <strong>Cartographer</strong> es el sistema SLAM desarrollado por Google, usado
        originalmente en sus robots de mapeo interior. Características destacadas:
        <ul>
          <li>
            <strong>Real-time loop closure</strong> → Detecta y corrige loops mientras mapea (no en
            post-procesamiento)
          </li>
          <li>
            <strong>Soporte 2D y 3D</strong> → Puede usar lidar 2D, 3D, IMU, odometría
            simultáneamente
          </li>
          <li>
            <strong>Submaps</strong> → Divide el mapa en regiones pequeñas para optimización
            eficiente
          </li>
          <li>
            <strong>Pose Graph Optimization</strong> con Ceres Solver (similar a SLAM Toolbox)
          </li>
        </ul>
      </TextBlock>

      <div class="code-section">
        <h4>Configuración de Cartographer para ROS 2</h4>

        <div class="code-block-container">
          <div class="code-header">
            <q-icon name="terminal" />
            <span>Instalación</span>
          </div>
          <pre><code class="language-bash"># Instalar Cartographer ROS
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros

# Crear configuración personalizada
mkdir -p ~/cartographer_ws/src
cd ~/cartographer_ws/src
git clone https://github.com/ros2/cartographer_ros.git -b humble
cd ~/cartographer_ws
colcon build
source install/setup.bash</code></pre>
        </div>

        <div class="code-block-container">
          <div class="code-header">
            <q-icon name="settings" />
            <span>Archivo de Configuración: my_robot.lua</span>
          </div>
          <pre><code class="language-lua">-- Copyright 2016 The Cartographer Authors
-- Licensed under the Apache License, Version 2.0

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,  -- 200 Hz
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- ==================== TRAJECTORY BUILDER 2D ====================
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 20.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)

-- Submaps
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- Scan Matching
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.0

-- ==================== POSE GRAPH ====================
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- Loop Closure
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.0)

-- Optimization
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options</code></pre>
        </div>

        <div class="code-block-container">
          <div class="code-header">
            <q-icon name="rocket_launch" />
            <span>Launch File: cartographer.launch.py</span>
          </div>
          <pre><code class="language-python">from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths
    cartographer_config_dir = os.path.join(
        get_package_share_directory('cartographer_ros'), 'configuration_files'
    )
    configuration_basename = 'my_robot.lua'

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )

    # Cartographer Node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/odom', '/odom'),
        ]
    )

    # Occupancy Grid Node (convierte submaps a nav_msgs/OccupancyGrid)
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'resolution': 0.05}
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)

    return ld</code></pre>
        </div>
      </div>

      <AlertBlock type="info" title="Cartographer vs SLAM Toolbox: ¿Cuál elegir?">
        <strong>Usa SLAM Toolbox si:</strong>
        <ul>
          <li>✅ Trabajas principalmente en 2D con lidar 2D</li>
          <li>✅ Necesitas lifelong mapping (actualizar mapas existentes)</li>
          <li>✅ Quieres integración nativa con Nav2</li>
          <li>✅ Prefieres configuración en YAML (más familiar para usuarios ROS)</li>
        </ul>
        <br />
        <strong>Usa Cartographer si:</strong>
        <ul>
          <li>✅ Necesitas SLAM 3D (lidar 3D, cámaras de profundidad)</li>
          <li>✅ Tienes múltiples sensores (lidar + IMU + odometría)</li>
          <li>✅ Requieres loop closure en tiempo real muy robusto</li>
          <li>✅ Trabajas en entornos muy grandes (almacenes, exteriores)</li>
        </ul>
        <br />
        <strong>Nota:</strong> Ambos usan Ceres Solver para optimización, así que la calidad del
        mapa final es comparable. La diferencia está en flexibilidad (Cartographer) vs simplicidad
        (SLAM Toolbox).
      </AlertBlock>
    </div>

    <!-- =====================================================================================
         BLOQUE G: CASE STUDY - IMPLEMENTACIÓN COMPLETA
         ===================================================================================== -->
    <div class="section-group">
      <SectionTitle>7. Case Study: Sistema SLAM Completo para Robot Móvil</SectionTitle>

      <TextBlock>
        Vamos a implementar un sistema SLAM completo desde cero, integrando:
        <ul>
          <li>Robot diferencial con lidar RPLidar A2</li>
          <li>SLAM Toolbox para mapping</li>
          <li>Nav2 para navegación autónoma</li>
          <li>Visualización en RViz</li>
        </ul>
      </TextBlock>

      <div class="code-section">
        <h4>Paso 1: Estructura del Paquete ROS 2</h4>

        <div class="code-block-container">
          <div class="code-header">
            <q-icon name="folder" />
            <span>Crear Paquete</span>
          </div>
          <pre><code class="language-bash"># Crear workspace
mkdir -p ~/slam_robot_ws/src
cd ~/slam_robot_ws/src

# Crear paquete
ros2 pkg create slam_robot \
  --build-type ament_cmake \
  --dependencies rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs tf2_ros

# Estructura de directorios
cd slam_robot
mkdir -p launch config maps rviz urdf
tree
# slam_robot/
# ├── config/
# │   ├── slam_params.yaml
# │   └── nav2_params.yaml
# ├── launch/
# │   ├── slam_launch.py
# │   └── navigation_launch.py
# ├── maps/
# ├── rviz/
# │   └── slam.rviz
# ├── urdf/
# │   └── robot.urdf.xacro
# ├── CMakeLists.txt
# └── package.xml</code></pre>
        </div>

        <h4>Paso 2: Descripción del Robot (URDF)</h4>

        <div class="code-block-container">
          <div class="code-header">
            <q-icon name="precision_manufacturing" />
            <span>urdf/robot.urdf.xacro</span>
          </div>
          <pre><code class="language-xml">&lt;?xml version="1.0"?&gt;
&lt;robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="slam_robot"&gt;

  &lt;!-- ==================== PROPIEDADES ==================== --&gt;
  &lt;xacro:property name="base_width" value="0.4"/&gt;
  &lt;xacro:property name="base_length" value="0.5"/&gt;
  &lt;xacro:property name="base_height" value="0.15"/&gt;
  &lt;xacro:property name="wheel_radius" value="0.1"/&gt;
  &lt;xacro:property name="wheel_width" value="0.05"/&gt;

  &lt;!-- ==================== BASE LINK ==================== --&gt;
  &lt;link name="base_link"&gt;
    &lt;visual&gt;
      &lt;geometry&gt;
        &lt;box size="${base_length} ${base_width} ${base_height}"/&gt;
      &lt;/geometry&gt;
      &lt;material name="blue"&gt;
        &lt;color rgba="0.2 0.4 0.8 1.0"/&gt;
      &lt;/material&gt;
    &lt;/visual&gt;
    &lt;collision&gt;
      &lt;geometry&gt;
        &lt;box size="${base_length} ${base_width} ${base_height}"/&gt;
      &lt;/geometry&gt;
    &lt;/collision&gt;
    &lt;inertial&gt;
      &lt;mass value="10.0"/&gt;
      &lt;inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/&gt;
    &lt;/inertial&gt;
  &lt;/link&gt;

  &lt;!-- ==================== LIDAR ==================== --&gt;
  &lt;link name="lidar_link"&gt;
    &lt;visual&gt;
      &lt;geometry&gt;
        &lt;cylinder radius="0.05" length="0.07"/&gt;
      &lt;/geometry&gt;
      &lt;material name="black"&gt;
        &lt;color rgba="0.1 0.1 0.1 1.0"/&gt;
      &lt;/material&gt;
    &lt;/visual&gt;
    &lt;collision&gt;
      &lt;geometry&gt;
        &lt;cylinder radius="0.05" length="0.07"/&gt;
      &lt;/geometry&gt;
    &lt;/collision&gt;
    &lt;inertial&gt;
      &lt;mass value="0.5"/&gt;
      &lt;inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/&gt;
    &lt;/inertial&gt;
  &lt;/link&gt;

  &lt;joint name="lidar_joint" type="fixed"&gt;
    &lt;parent link="base_link"/&gt;
    &lt;child link="lidar_link"/&gt;
    &lt;origin xyz="0.0 0.0 ${base_height/2 + 0.035}" rpy="0 0 0"/&gt;
  &lt;/joint&gt;

  &lt;!-- ==================== GAZEBO PLUGINS ==================== --&gt;
  &lt;gazebo reference="lidar_link"&gt;
    &lt;sensor name="lidar" type="ray"&gt;
      &lt;always_on&gt;true&lt;/always_on&gt;
      &lt;visualize&gt;true&lt;/visualize&gt;
      &lt;update_rate&gt;10&lt;/update_rate&gt;
      &lt;ray&gt;
        &lt;scan&gt;
          &lt;horizontal&gt;
            &lt;samples&gt;360&lt;/samples&gt;
            &lt;resolution&gt;1.0&lt;/resolution&gt;
            &lt;min_angle&gt;-3.14159&lt;/min_angle&gt;
            &lt;max_angle&gt;3.14159&lt;/max_angle&gt;
          &lt;/horizontal&gt;
        &lt;/scan&gt;
        &lt;range&gt;
          &lt;min&gt;0.12&lt;/min&gt;
          &lt;max&gt;30.0&lt;/max&gt;
          &lt;resolution&gt;0.015&lt;/resolution&gt;
        &lt;/range&gt;
      &lt;/ray&gt;
      &lt;plugin name="scan" filename="libgazebo_ros_ray_sensor.so"&gt;
        &lt;ros&gt;
          &lt;remapping&gt;~/out:=scan&lt;/remapping&gt;
        &lt;/ros&gt;
        &lt;output_type&gt;sensor_msgs/LaserScan&lt;/output_type&gt;
        &lt;frame_name&gt;lidar_link&lt;/frame_name&gt;
      &lt;/plugin&gt;
    &lt;/sensor&gt;
  &lt;/gazebo&gt;

  &lt;!-- Differential Drive Plugin --&gt;
  &lt;gazebo&gt;
    &lt;plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so"&gt;
      &lt;update_rate&gt;50&lt;/update_rate&gt;
      &lt;left_joint&gt;left_wheel_joint&lt;/left_joint&gt;
      &lt;right_joint&gt;right_wheel_joint&lt;/right_joint&gt;
      &lt;wheel_separation&gt;${base_width}&lt;/wheel_separation&gt;
      &lt;wheel_diameter&gt;${2*wheel_radius}&lt;/wheel_diameter&gt;
      &lt;max_wheel_torque&gt;20&lt;/max_wheel_torque&gt;
      &lt;command_topic&gt;cmd_vel&lt;/command_topic&gt;
      &lt;publish_odom&gt;true&lt;/publish_odom&gt;
      &lt;publish_odom_tf&gt;true&lt;/publish_odom_tf&gt;
      &lt;publish_wheel_tf&gt;false&lt;/publish_wheel_tf&gt;
      &lt;odometry_topic&gt;odom&lt;/odometry_topic&gt;
      &lt;odometry_frame&gt;odom&lt;/odometry_frame&gt;
      &lt;robot_base_frame&gt;base_link&lt;/robot_base_frame&gt;
    &lt;/plugin&gt;
  &lt;/gazebo&gt;

&lt;/robot&gt;</code></pre>
        </div>

        <h4>Paso 3: Launch File Completo</h4>

        <div class="code-block-container">
          <div class="code-header">
            <q-icon name="rocket_launch" />
            <span>launch/slam_launch.py</span>
          </div>
          <pre><code class="language-python">import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Package directories
    pkg_slam_robot = FindPackageShare('slam_robot')
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    pkg_slam_toolbox = FindPackageShare('slam_toolbox')

    # Paths
    urdf_file = PathJoinSubstitution([pkg_slam_robot, 'urdf', 'robot.urdf.xacro'])
    slam_params = PathJoinSubstitution([pkg_slam_robot, 'config', 'slam_params.yaml'])
    rviz_config = PathJoinSubstitution([pkg_slam_robot, 'rviz', 'slam.rviz'])
    world_file = PathJoinSubstitution([pkg_gazebo_ros, 'worlds', 'empty.world'])

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={'world': world_file}.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': urdf_file}
        ]
    )

    # Spawn Robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'slam_robot', '-topic', 'robot_description'],
        output='screen'
    )

    # SLAM Toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': use_sim_time}]
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        slam_toolbox,
        rviz
    ])</code></pre>
        </div>

        <h4>Paso 4: Ejecución y Resultados</h4>

        <div class="code-block-container">
          <div class="code-header">
            <q-icon name="play_arrow" />
            <span>Comandos de Ejecución</span>
          </div>
          <pre><code class="language-bash"># Compilar el workspace
cd ~/slam_robot_ws
colcon build --symlink-install
source install/setup.bash

# Lanzar el sistema completo
ros2 launch slam_robot slam_launch.py

# En otra terminal, teleoperar el robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Guardar el mapa cuando termines
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: my_map}}"

# Verificar topics activos
ros2 topic list
# /scan
# /odom
# /map
# /tf
# /tf_static
# /cmd_vel

# Monitorear calidad del SLAM
ros2 topic echo /slam_toolbox/graph_visualization</code></pre>
        </div>
      </div>

      <AlertBlock type="success" title="Resultados Esperados">
        Después de explorar el entorno durante 5-10 minutos, deberías obtener:
        <br /><br />
        <strong>✅ Mapa 2D consistente</strong> → Sin duplicaciones ni "fantasmas"
        <br />
        <strong>✅ Loop closures detectados</strong> → El mapa se corrige cuando regresas a un lugar
        conocido
        <br />
        <strong>✅ Pose graph optimizado</strong> → Trayectoria suave y consistente
        <br />
        <strong>✅ Archivos generados:</strong>
        <ul>
          <li><code>my_map.yaml</code> → Metadatos del mapa</li>
          <li><code>my_map.pgm</code> → Imagen del mapa (occupancy grid)</li>
          <li><code>my_map.posegraph</code> → Grafo de poses (para lifelong mapping)</li>
        </ul>
      </AlertBlock>
    </div>

    <!-- =====================================================================================
         SUMMARY & NEXT STEPS
         ===================================================================================== -->
    <div class="section-group">
      <SectionTitle>Resumen y Próximos Pasos</SectionTitle>

      <TextBlock>
        En esta lección hemos cubierto:
        <ul>
          <li>
            <strong>Fundamentos matemáticos</strong> → Teorema de Bayes, Particle Filter, Graph-SLAM
          </li>
          <li>
            <strong>Algoritmos</strong> → FastSLAM, Pose Graph Optimization, Loop Closure Detection
          </li>
          <li>
            <strong>Sensores</strong> → Comparación de Lidar 2D/3D, RGB-D, cámaras monoculares
          </li>
          <li>
            <strong>Implementaciones ROS 2</strong> → SLAM Toolbox y Cartographer con configuración
            completa
          </li>
          <li><strong>Case Study</strong> → Sistema SLAM funcional desde cero</li>
        </ul>
      </TextBlock>

      <AlertBlock type="info" title="Próxima Lección: Localización (AMCL)">
        Una vez que tienes un mapa, el siguiente paso es <strong>localizar</strong> el robot en ese
        mapa. En la próxima lección aprenderás:
        <ul>
          <li>
            <strong>AMCL (Adaptive Monte Carlo Localization)</strong> → Particle filter para
            localización
          </li>
          <li><strong>Tuning de parámetros</strong> → Cómo ajustar AMCL para tu robot</li>
          <li>
            <strong>Integración con Nav2</strong> → Usar el mapa y la localización para navegación
            autónoma
          </li>
        </ul>
      </AlertBlock>

      <div class="resource-grid">
        <div class="resource-card">
          <q-icon name="book" size="2rem" color="blue-4" />
          <h4>Recursos Adicionales</h4>
          <ul>
            <li>
              <a
                href="https://github.com/SteveMacenski/slam_toolbox"
                target="_blank"
                rel="noopener noreferrer"
              >
                SLAM Toolbox GitHub
              </a>
            </li>
            <li>
              <a
                href="https://google-cartographer-ros.readthedocs.io/"
                target="_blank"
                rel="noopener noreferrer"
              >
                Cartographer Documentation
              </a>
            </li>
            <li>
              <a
                href="http://www.probabilistic-robotics.org/"
                target="_blank"
                rel="noopener noreferrer"
              >
                Probabilistic Robotics (Thrun et al.)
              </a>
            </li>
          </ul>
        </div>

        <div class="resource-card">
          <q-icon name="video_library" size="2rem" color="red-4" />
          <h4>Videos Recomendados</h4>
          <ul>
            <li>Cyrill Stachniss - SLAM Course (YouTube)</li>
            <li>Steve Macenski - SLAM Toolbox Tutorial (ROSCon 2019)</li>
            <li>Google Cartographer Demo (YouTube)</li>
          </ul>
        </div>

        <div class="resource-card">
          <q-icon name="code" size="2rem" color="green-4" />
          <h4>Código de Ejemplo</h4>
          <ul>
            <li>
              <a
                href="https://github.com/ros-planning/navigation2"
                target="_blank"
                rel="noopener noreferrer"
              >
                Nav2 Examples
              </a>
            </li>
            <li>
              <a
                href="https://github.com/ROBOTIS-GIT/turtlebot3"
                target="_blank"
                rel="noopener noreferrer"
              >
                TurtleBot3 SLAM Examples
              </a>
            </li>
          </ul>
        </div>
      </div>
    </div>
  </LessonContainer>
</template>

<script setup lang="ts">
import LessonContainer from 'components/content/LessonContainer.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
</script>

<style scoped>
/* ========================================================================================
   HERO SECTION
   ======================================================================================== */
.hero-section {
  background:
    radial-gradient(circle at 80% 20%, rgba(59, 130, 246, 0.15), transparent 50%),
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
  background: rgba(59, 130, 246, 0.15);
  color: #60a5fa;
  padding: 0.5rem 1rem;
  border-radius: 99px;
  font-size: 0.85rem;
  font-weight: 700;
  border: 1px solid rgba(59, 130, 246, 0.3);
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
  background: linear-gradient(to right, #60a5fa, #22d3ee);
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

.stat-item {
  display: flex;
  flex-direction: column;
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

.slam-animation {
  width: 100%;
  height: 100%;
}

/* Animations */
@keyframes robot-move {
  0%,
  100% {
    transform: translate(150px, 150px) rotate(0deg);
  }
  25% {
    transform: translate(120px, 120px) rotate(45deg);
  }
  50% {
    transform: translate(180px, 120px) rotate(90deg);
  }
  75% {
    transform: translate(180px, 180px) rotate(135deg);
  }
}

@keyframes grid-build {
  0% {
    opacity: 0;
    stroke-dashoffset: 200;
  }
  100% {
    opacity: 0.2;
    stroke-dashoffset: 0;
  }
}

@keyframes uncertainty-pulse {
  0%,
  100% {
    opacity: 0.2;
    r: 50;
  }
  50% {
    opacity: 0.4;
    r: 70;
  }
}

.robot {
  animation: robot-move 8s ease-in-out infinite;
}

.grid-line {
  stroke-dasharray: 200;
  animation: grid-build 2s ease-out forwards;
}

.g1 {
  animation-delay: 0.2s;
}
.g2 {
  animation-delay: 0.4s;
}
.g3 {
  animation-delay: 0.6s;
}
.g4 {
  animation-delay: 0.8s;
}
.g5 {
  animation-delay: 1s;
}
.g6 {
  animation-delay: 1.2s;
}

.uncertainty {
  animation: uncertainty-pulse 3s ease-in-out infinite;
}

.trajectory {
  stroke-dasharray: 100;
  stroke-dashoffset: 100;
  animation: draw-trajectory 4s ease-out forwards;
}

@keyframes draw-trajectory {
  to {
    stroke-dashoffset: 0;
  }
}

/* ========================================================================================
   CONTENT BLOCKS
   ======================================================================================== */
.content-block {
  margin-bottom: 4rem;
}

/* MATH FOUNDATION */
.math-foundation {
  margin: 2rem 0;
}

.foundation-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  overflow: hidden;
}

.foundation-header {
  background: rgba(139, 92, 246, 0.1);
  padding: 1.5rem 2rem;
  display: flex;
  align-items: center;
  gap: 1rem;
  border-bottom: 1px solid rgba(148, 163, 184, 0.1);
}

.foundation-header span {
  font-size: 1.25rem;
  font-weight: 700;
  color: #c4b5fd;
}

.foundation-body {
  padding: 2rem;
}

.equation-block {
  margin-bottom: 2rem;
}

.equation-block:last-child {
  margin-bottom: 0;
}

.eq-title {
  font-size: 1.1rem;
  font-weight: 600;
  color: #94a3b8;
  margin-bottom: 1rem;
}

.eq-formula {
  font-size: 1.3rem;
  color: #e2e8f0;
  padding: 1.5rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 12px;
  margin-bottom: 1rem;
  text-align: center;
  font-family: 'Cambria Math', 'Latin Modern Math', serif;
  border: 1px solid rgba(148, 163, 184, 0.1);
}

.eq-desc {
  font-size: 1rem;
  color: #cbd5e1;
  line-height: 1.8;
}

.eq-desc ul {
  margin-top: 0.75rem;
  padding-left: 1.5rem;
}

.eq-desc li {
  margin-bottom: 0.75rem;
  color: #94a3b8;
}

.eq-desc strong {
  color: #e2e8f0;
}

/* ========================================================================================
   RESPONSIVE
   ======================================================================================== */
@media (max-width: 1024px) {
  .hero-section {
    flex-direction: column;
    padding: 3rem 2rem;
  }

  .hero-viz {
    width: 300px;
    height: 300px;
  }

  .hero-title {
    font-size: 2.5rem;
  }
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2rem;
  }

  .hero-stats {
    flex-direction: column;
    gap: 1.5rem;
  }

  .hero-viz {
    width: 250px;
    height: 250px;
  }
}

/* ========================================================================================
   CODE SECTION STYLES
   ======================================================================================== */
.code-section {
  margin-top: 2rem;
}

.code-section h4 {
  font-size: 1.4rem;
  font-weight: 700;
  color: #e2e8f0;
  margin-bottom: 1.5rem;
  padding-bottom: 0.75rem;
  border-bottom: 2px solid rgba(59, 130, 246, 0.3);
}

.code-block-container {
  background: linear-gradient(135deg, #1e293b 0%, #0f172a 100%);
  border-radius: 16px;
  overflow: hidden;
  margin-bottom: 2rem;
  border: 1px solid rgba(148, 163, 184, 0.15);
  box-shadow: 0 10px 30px -5px rgba(0, 0, 0, 0.4);
}

.code-header {
  background: linear-gradient(90deg, rgba(59, 130, 246, 0.2), rgba(147, 51, 234, 0.2));
  padding: 1rem 1.5rem;
  display: flex;
  align-items: center;
  gap: 0.75rem;
  border-bottom: 1px solid rgba(148, 163, 184, 0.1);
}

.code-header span {
  font-size: 1rem;
  font-weight: 600;
  color: #e2e8f0;
}

.code-block-container pre {
  margin: 0;
  padding: 1.5rem;
  overflow-x: auto;
  background: transparent;
}

.code-block-container code {
  font-family: 'Fira Code', 'Consolas', 'Monaco', monospace;
  font-size: 0.9rem;
  line-height: 1.6;
  color: #cbd5e1;
}

/* ========================================================================================
   SENSOR COMPARISON GRID
   ======================================================================================== */
.sensor-comparison-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 2rem;
  margin-top: 2rem;
}

.sensor-card {
  background: linear-gradient(135deg, #1e293b 0%, #0f172a 100%);
  border-radius: 20px;
  overflow: hidden;
  border: 1px solid rgba(148, 163, 184, 0.15);
  box-shadow: 0 10px 30px -5px rgba(0, 0, 0, 0.4);
  transition: all 0.3s ease;
}

.sensor-card:hover {
  transform: translateY(-5px);
  box-shadow: 0 20px 40px -10px rgba(0, 0, 0, 0.6);
  border-color: rgba(59, 130, 246, 0.4);
}

.sensor-header {
  padding: 2rem 1.5rem 1.5rem;
  text-align: center;
  background: linear-gradient(135deg, rgba(59, 130, 246, 0.1), rgba(147, 51, 234, 0.1));
  border-bottom: 1px solid rgba(148, 163, 184, 0.1);
}

.sensor-header h3 {
  font-size: 1.5rem;
  font-weight: 700;
  color: #e2e8f0;
  margin: 1rem 0 0.5rem;
}

.sensor-badge {
  display: inline-block;
  padding: 0.4rem 1rem;
  background: rgba(59, 130, 246, 0.2);
  border-radius: 20px;
  font-size: 0.85rem;
  font-weight: 600;
  color: #60a5fa;
  border: 1px solid rgba(59, 130, 246, 0.3);
}

.sensor-body {
  padding: 1.5rem;
}

.sensor-spec {
  padding: 0.75rem 0;
  border-bottom: 1px solid rgba(148, 163, 184, 0.05);
  font-size: 0.95rem;
  color: #cbd5e1;
}

.sensor-spec strong {
  color: #e2e8f0;
  font-weight: 600;
}

.sensor-pros,
.sensor-cons {
  margin-top: 1rem;
}

.sensor-pros strong,
.sensor-cons strong {
  display: block;
  margin-bottom: 0.5rem;
  color: #e2e8f0;
  font-size: 1rem;
}

.sensor-pros ul,
.sensor-cons ul {
  list-style: none;
  padding: 0;
  margin: 0;
}

.sensor-pros li,
.sensor-cons li {
  padding: 0.4rem 0;
  font-size: 0.9rem;
  color: #94a3b8;
}

.sensor-examples {
  margin-top: 1rem;
  padding-top: 1rem;
  border-top: 1px solid rgba(148, 163, 184, 0.1);
  font-size: 0.9rem;
  color: #94a3b8;
}

.sensor-examples strong {
  color: #e2e8f0;
}

/* Sensor-specific colors */
.sensor-card.lidar-2d .sensor-header {
  background: linear-gradient(135deg, rgba(59, 130, 246, 0.15), rgba(37, 99, 235, 0.1));
}

.sensor-card.lidar-3d .sensor-header {
  background: linear-gradient(135deg, rgba(147, 51, 234, 0.15), rgba(126, 34, 206, 0.1));
}

.sensor-card.rgbd .sensor-header {
  background: linear-gradient(135deg, rgba(16, 185, 129, 0.15), rgba(5, 150, 105, 0.1));
}

.sensor-card.monocular .sensor-header {
  background: linear-gradient(135deg, rgba(245, 158, 11, 0.15), rgba(217, 119, 6, 0.1));
}

/* ========================================================================================
   RESOURCE GRID
   ======================================================================================== */
.resource-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  gap: 2rem;
  margin-top: 2rem;
}

.resource-card {
  background: linear-gradient(135deg, #1e293b 0%, #0f172a 100%);
  border-radius: 16px;
  padding: 2rem;
  border: 1px solid rgba(148, 163, 184, 0.15);
  box-shadow: 0 10px 30px -5px rgba(0, 0, 0, 0.4);
  transition: all 0.3s ease;
}

.resource-card:hover {
  transform: translateY(-3px);
  box-shadow: 0 15px 35px -8px rgba(0, 0, 0, 0.5);
  border-color: rgba(59, 130, 246, 0.3);
}

.resource-card h4 {
  font-size: 1.2rem;
  font-weight: 700;
  color: #e2e8f0;
  margin: 1rem 0 1.5rem;
}

.resource-card ul {
  list-style: none;
  padding: 0;
  margin: 0;
}

.resource-card li {
  padding: 0.5rem 0;
  font-size: 0.95rem;
  color: #94a3b8;
  border-bottom: 1px solid rgba(148, 163, 184, 0.05);
}

.resource-card li:last-child {
  border-bottom: none;
}

.resource-card a {
  color: #60a5fa;
  text-decoration: none;
  transition: color 0.2s ease;
}

.resource-card a:hover {
  color: #93c5fd;
  text-decoration: underline;
}

/* ========================================================================================
   SECTION GROUP
   ======================================================================================== */
.section-group {
  margin-bottom: 4rem;
}

/* ========================================================================================
   FORMULA STYLING
   ======================================================================================== */
.formula {
  font-size: 1.2rem;
  color: #e2e8f0;
  padding: 1rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 8px;
  margin: 0.5rem 0;
  text-align: center;
  font-family: 'Cambria Math', 'Latin Modern Math', serif;
  border: 1px solid rgba(148, 163, 184, 0.1);
}
</style>
