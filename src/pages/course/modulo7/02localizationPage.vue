<template>
  <LessonContainer>
    <!-- ========================================================================================
         HERO SECTION: BAYESIAN LOCALIZATION
         Visual: Particle cloud convergence + probability waves
         ======================================================================================== -->
    <div class="hero-section text-white q-mb-xl">
      <div class="hero-content">
        <div class="hero-badge"><q-icon name="explore" /> MÓDULO 7.2: LOCALIZATION</div>
        <h1 class="hero-title">
          AMCL: <span class="gradient-text">Probabilistic Localization</span>
        </h1>
        <p class="hero-subtitle">
          La localización no es un problema de geometría, es un problema de
          <strong>inferencia probabilística</strong>. El robot nunca "sabe" dónde está; mantiene una
          <em>distribución de creencias</em> que colapsa hacia la verdad mediante observación y
          movimiento. Bienvenido al mundo de los Filtros de Partículas.
        </p>

        <div class="hero-stats">
          <div class="stat-item">
            <div class="stat-val">5000</div>
            <div class="stat-label">Partículas Activas</div>
          </div>
          <div class="stat-item">
            <div class="stat-val">99.9%</div>
            <div class="stat-label">Convergencia</div>
          </div>
          <div class="stat-item">
            <div class="stat-val">KLD</div>
            <div class="stat-label">Adaptive Sampling</div>
          </div>
        </div>
      </div>

      <div class="hero-viz">
        <!-- Particle Cloud Animation -->
        <svg viewBox="0 0 300 300" class="particle-cloud">
          <defs>
            <radialGradient id="particleGrad">
              <stop offset="0%" stop-color="#ef4444" stop-opacity="1" />
              <stop offset="100%" stop-color="#ef4444" stop-opacity="0" />
            </radialGradient>
            <filter id="glow">
              <feGaussianBlur stdDeviation="3" result="coloredBlur" />
              <feMerge>
                <feMergeNode in="coloredBlur" />
                <feMergeNode in="SourceGraphic" />
              </feMerge>
            </filter>
          </defs>

          <!-- Robot Position (Ground Truth) -->
          <circle cx="150" cy="150" r="8" fill="#10b981" filter="url(#glow)" class="robot-pos" />
          <circle
            cx="150"
            cy="150"
            r="15"
            fill="none"
            stroke="#10b981"
            stroke-width="2"
            opacity="0.3"
            class="robot-ring"
          />

          <!-- Particle Swarm -->
          <g class="particles">
            <circle
              v-for="i in 50"
              :key="i"
              :cx="150 + Math.cos((i / 50) * Math.PI * 2) * (30 + Math.random() * 40)"
              :cy="150 + Math.sin((i / 50) * Math.PI * 2) * (30 + Math.random() * 40)"
              r="2"
              fill="url(#particleGrad)"
              class="particle"
              :style="{ animationDelay: `${i * 0.05}s` }"
            />
          </g>

          <!-- Probability Density Waves -->
          <circle
            cx="150"
            cy="150"
            r="50"
            fill="none"
            stroke="#3b82f6"
            stroke-width="1"
            opacity="0.2"
            class="prob-wave w1"
          />
          <circle
            cx="150"
            cy="150"
            r="70"
            fill="none"
            stroke="#3b82f6"
            stroke-width="1"
            opacity="0.15"
            class="prob-wave w2"
          />
          <circle
            cx="150"
            cy="150"
            r="90"
            fill="none"
            stroke="#3b82f6"
            stroke-width="1"
            opacity="0.1"
            class="prob-wave w3"
          />
        </svg>
      </div>
    </div>

    <!-- ========================================================================================
         BLOCK A: BAYESIAN FILTER THEORY (DOCTORAL DEPTH)
         Mathematical foundations of probabilistic robotics
         ======================================================================================== -->
    <div class="content-block">
      <SectionTitle>1. Fundamentos del Filtro Bayesiano</SectionTitle>

      <TextBlock>
        La localización probabilística se basa en el <strong>Teorema de Bayes</strong> aplicado
        recursivamente en el tiempo. El robot mantiene una <em>creencia</em> (belief) sobre su
        estado, que se actualiza en dos fases: <strong>Predicción</strong> (motion model) y
        <strong>Corrección</strong> (sensor model).
      </TextBlock>

      <AlertBlock type="info" title="Filosofía Bayesiana">
        <strong>"¿Dónde estoy?"</strong> no tiene una respuesta única. En su lugar, el robot
        responde:
        <br />
        <em
          >"Hay un 85% de probabilidad de que esté en la cocina, 10% en el pasillo, y 5% distribuido
          en otras habitaciones"</em
        >
        <br /><br />
        Esta distribución de probabilidad es la <code>Belief: Bel(x_t)</code>
      </AlertBlock>

      <!-- BAYES FILTER EQUATIONS -->
      <div class="theory-section q-my-lg">
        <div class="theory-header">
          <q-icon name="functions" size="lg" />
          <span>Ecuaciones del Filtro Bayesiano</span>
        </div>

        <div class="equations-grid">
          <!-- PREDICTION STEP -->
          <div class="equation-card prediction">
            <div class="eq-title">
              <q-icon name="directions_run" />
              <span>Paso 1: Predicción (Motion Update)</span>
            </div>
            <div class="eq-formula">
              <MathFormula
                :formula="'\\overline{Bel}(x_t) = \\int p(x_t | u_t, x_{t-1}) \\cdot Bel(x_{t-1}) \\, dx_{t-1}'"
              />
            </div>
            <div class="eq-explanation">
              <strong>Interpretación:</strong> Propagamos la creencia anterior
              <MathFormula :formula="'Bel(x_{t-1})'" inline /> a través del modelo de movimiento
              <MathFormula :formula="'p(x_t | u_t, x_{t-1})'" inline />, donde
              <MathFormula :formula="'u_t'" inline /> es el comando de control (odometría).
              <br /><br />
              <strong>Efecto:</strong> La incertidumbre <em>aumenta</em> (la nube de partículas se
              dispersa) porque el movimiento introduce ruido.
            </div>
            <div class="eq-params">
              <div class="param-item">
                <code>x_t</code>
                <span>Estado en tiempo t (posición + orientación)</span>
              </div>
              <div class="param-item">
                <code>u_t</code>
                <span>Control de odometría (Δx, Δy, Δθ)</span>
              </div>
              <div class="param-item">
                <code>p(x_t | u_t, x_{t-1})</code>
                <span>Modelo de movimiento (probabilidad de transición)</span>
              </div>
            </div>
          </div>

          <!-- CORRECTION STEP -->
          <div class="equation-card correction">
            <div class="eq-title">
              <q-icon name="visibility" />
              <span>Paso 2: Corrección (Sensor Update)</span>
            </div>
            <div class="eq-formula">
              <MathFormula
                :formula="'Bel(x_t) = \\eta \\cdot p(z_t | x_t) \\cdot \\overline{Bel}(x_t)'"
              />
            </div>
            <div class="eq-explanation">
              <strong>Interpretación:</strong> Multiplicamos la predicción
              <MathFormula :formula="'\\overline{Bel}(x_t)'" inline /> por la verosimilitud del
              sensor <MathFormula :formula="'p(z_t | x_t)'" inline />, donde
              <MathFormula :formula="'z_t'" inline /> es la observación (escaneo láser).
              <MathFormula :formula="'\\eta'" inline /> es un factor de normalización. <br /><br />
              <strong>Efecto:</strong> La incertidumbre <em>disminuye</em> (la nube se concentra)
              porque la observación descarta hipótesis incompatibles.
            </div>
            <div class="eq-params">
              <div class="param-item">
                <code>z_t</code>
                <span>Observación del sensor (LaserScan)</span>
              </div>
              <div class="param-item">
                <code>p(z_t | x_t)</code>
                <span>Modelo de sensor (likelihood)</span>
              </div>
              <div class="param-item">
                <code>η</code>
                <span>Constante de normalización: η = 1 / ∫ p(z_t | x_t) Bel̄(x_t) dx_t</span>
              </div>
            </div>
          </div>
        </div>
      </div>

      <!-- MONTE CARLO APPROXIMATION -->
      <div class="mc-section q-mt-xl">
        <div class="section-subtitle">
          <q-icon name="casino" />
          <span>Aproximación de Monte Carlo (Particle Filter)</span>
        </div>

        <TextBlock>
          El problema del Filtro Bayesiano es que la integral es <strong>intratable</strong> para
          espacios de estado continuos. La solución:
          <strong>Monte Carlo Localization (MCL)</strong>, que aproxima la distribución continua con
          un conjunto discreto de <em>partículas</em>.
        </TextBlock>

        <div class="mc-concept-grid q-my-md">
          <div class="concept-card">
            <div class="concept-icon">🎲</div>
            <div class="concept-title">Representación Discreta</div>
            <div class="concept-desc">
              En lugar de <MathFormula :formula="'Bel(x_t)'" inline /> continua, usamos un conjunto
              de <MathFormula :formula="'N'" inline /> partículas:
              <br />
              <MathFormula
                :formula="'\\chi_t = \\{x_t^{[1]}, x_t^{[2]}, ..., x_t^{[N]}\\}'"
                class="q-my-sm"
              />
            </div>
          </div>

          <div class="concept-card">
            <div class="concept-icon">⚖️</div>
            <div class="concept-title">Pesos de Importancia</div>
            <div class="concept-desc">
              Cada partícula tiene un peso <MathFormula :formula="'w_t^{[i]}'" inline /> que
              representa su probabilidad. La suma de todos los pesos es 1.
            </div>
          </div>

          <div class="concept-card">
            <div class="concept-icon">♻️</div>
            <div class="concept-title">Resampling</div>
            <div class="concept-desc">
              Periódicamente, eliminamos partículas de bajo peso y duplicamos las de alto peso
              (Survival of the Fittest).
            </div>
          </div>
        </div>

        <!-- MCL ALGORITHM PSEUDOCODE -->
        <div class="algorithm-box q-mt-lg">
          <div class="algo-header">
            <q-icon name="code" />
            <span>Algoritmo: Monte Carlo Localization (MCL)</span>
          </div>
          <div class="algo-content">
            <div class="pseudocode-container">
              <div class="code-line">
                <span class="keyword">Algorithm</span>
                <span class="func-name">MCL</span>(<MathFormula
                  :formula="'\\chi_{t-1}, u_t, z_t'"
                  inline
                />):
              </div>

              <div class="code-line indent-1">
                <span class="comment">// INPUT: Partículas previas, control, observación</span>
              </div>
              <div class="code-line indent-1">
                <span class="comment"
                  >// OUTPUT: Nuevas partículas <MathFormula :formula="'\\chi_t'" inline
                /></span>
              </div>
              <div class="code-line spacer"></div>

              <div class="code-line indent-1">
                <MathFormula :formula="'\\bar{\\chi}_t = \\emptyset'" inline />
                <span class="comment q-ml-sm">// Conjunto temporal vacío</span>
              </div>
              <div class="code-line indent-1">
                <MathFormula :formula="'\\chi_t = \\emptyset'" inline />
              </div>
              <div class="code-line spacer"></div>

              <div class="code-line indent-1">
                <span class="keyword">for</span> <MathFormula :formula="'i = 1'" inline />
                <span class="keyword">to</span> <MathFormula :formula="'N'" inline />:
              </div>

              <div class="code-line indent-2">
                <span class="comment"
                  >// 1. PREDICTION: Sample nuevo estado desde motion model</span
                >
              </div>
              <div class="code-line indent-2">
                <MathFormula
                  :formula="'x_t^{[i]} = \\text{sample\\_motion\\_model}(u_t, x_{t-1}^{[i]})'"
                  inline
                />
              </div>
              <div class="code-line spacer"></div>

              <div class="code-line indent-2">
                <span class="comment">// 2. CORRECTION: Calcular peso basado en sensor model</span>
              </div>
              <div class="code-line indent-2">
                <MathFormula :formula="'w_t^{[i]} = p(z_t | x_t^{[i]}, map)'" inline />
              </div>
              <div class="code-line spacer"></div>

              <div class="code-line indent-2">
                <span class="comment">// Agregar a conjunto temporal</span>
              </div>
              <div class="code-line indent-2">
                <MathFormula
                  :formula="'\\bar{\\chi}_t = \\bar{\\chi}_t \\cup \\{ \\langle x_t^{[i]}, w_t^{[i]} \\rangle \\}'"
                  inline
                />
              </div>

              <div class="code-line indent-1">
                <span class="keyword">end for</span>
              </div>
              <div class="code-line spacer"></div>

              <div class="code-line indent-1">
                <span class="comment">// 3. RESAMPLING: Selección proporcional a pesos</span>
              </div>
              <div class="code-line indent-1">
                <span class="keyword">for</span> <MathFormula :formula="'i = 1'" inline />
                <span class="keyword">to</span> <MathFormula :formula="'N'" inline />:
              </div>
              <div class="code-line indent-2">
                draw <MathFormula :formula="'j'" inline /> with probability
                <MathFormula :formula="'\\propto w_t^{[j]}'" inline />
              </div>
              <div class="code-line indent-2">
                <MathFormula :formula="'\\chi_t = \\chi_t \\cup \\{ x_t^{[j]} \\}'" inline />
              </div>
              <div class="code-line indent-1">
                <span class="keyword">end for</span>
              </div>
              <div class="code-line spacer"></div>

              <div class="code-line indent-1">
                <span class="keyword">return</span> <MathFormula :formula="'\\chi_t'" inline />
              </div>
            </div>
          </div>
        </div>
      </div>

      <SectionTitle class="q-mt-xl">2. Implementación del Motion Model</SectionTitle>

      <TextBlock>
        El modelo de movimiento define cómo las partículas se propagan cuando el robot se mueve.
        AMCL usa el <strong>Sample Odometry Motion Model</strong>, que añade ruido gaussiano a los
        parámetros de odometría.
      </TextBlock>

      <!-- Motion Model Code -->
      <CodeBlock
        lang="cpp"
        title="motion_model.cpp - Implementación Completa"
        :copyable="true"
        content="/**
 * @file motion_model.cpp
 * @brief Sample Odometry Motion Model para AMCL
 * @details Implementa el algoritmo de Probabilistic Robotics (Thrun et al.)
 *          Capítulo 5.4: Sample Odometry Motion Model
 */

#include <random>
#include <cmath>

class SampleOdometryMotionModel {
public:
  /**
   * @brief Constructor con parámetros de ruido
   * @param alpha1 Error de rotación causado por rotación (rot→rot)
   * @param alpha2 Error de rotación causado por traslación (trans→rot)
   * @param alpha3 Error de traslación causado por traslación (trans→trans)
   * @param alpha4 Error de traslación causado por rotación (rot→trans)
   */
  SampleOdometryMotionModel(double alpha1, double alpha2,
                            double alpha3, double alpha4)
    : alpha1_(alpha1), alpha2_(alpha2),
      alpha3_(alpha3), alpha4_(alpha4),
      generator_(std::random_device{}()) {}

  /**
   * @brief Muestrea una nueva pose basada en odometría + ruido
   * @param x_prev Pose anterior [x, y, theta]
   * @param u_odom Odometría [x1, y1, theta1, x2, y2, theta2]
   * @return Nueva pose muestreada
   */
  Pose sample(const Pose& x_prev, const Odometry& u_odom) {
    // 1. Extraer parámetros de odometría
    double delta_rot1 = atan2(u_odom.y2 - u_odom.y1,
                               u_odom.x2 - u_odom.x1) - u_odom.theta1;
    double delta_trans = sqrt(pow(u_odom.x2 - u_odom.x1, 2) +
                              pow(u_odom.y2 - u_odom.y1, 2));
    double delta_rot2 = u_odom.theta2 - u_odom.theta1 - delta_rot1;

    // 2. Añadir ruido gaussiano (Sample from Normal Distribution)
    double delta_rot1_noisy = delta_rot1 -
      sample_normal(alpha1_ * delta_rot1 * delta_rot1 +
                    alpha2_ * delta_trans * delta_trans);

    double delta_trans_noisy = delta_trans -
      sample_normal(alpha3_ * delta_trans * delta_trans +
                    alpha4_ * delta_rot1 * delta_rot1 +
                    alpha4_ * delta_rot2 * delta_rot2);

    double delta_rot2_noisy = delta_rot2 -
      sample_normal(alpha1_ * delta_rot2 * delta_rot2 +
                    alpha2_ * delta_trans * delta_trans);

    // 3. Aplicar movimiento ruidoso a la pose anterior
    Pose x_new;
    x_new.x = x_prev.x + delta_trans_noisy * cos(x_prev.theta + delta_rot1_noisy);
    x_new.y = x_prev.y + delta_trans_noisy * sin(x_prev.theta + delta_rot1_noisy);
    x_new.theta = normalize_angle(x_prev.theta + delta_rot1_noisy + delta_rot2_noisy);

    return x_new;
  }

private:
  double alpha1_, alpha2_, alpha3_, alpha4_;  // Noise parameters
  std::mt19937 generator_;

  /**
   * @brief Muestrea de una distribución normal con varianza b
   * @param b Varianza (b = σ²)
   * @return Muestra aleatoria
   */
  double sample_normal(double b) {
    std::normal_distribution<double> dist(0.0, sqrt(b));
    return dist(generator_);
  }

  /**
   * @brief Normaliza ángulo al rango [-π, π]
   */
  double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }
};"
      />
    </div>

    <!-- ========================================================================================
         BLOCK C: SENSOR MODEL (LIKELIHOOD FIELD)
         Depth: Probabilistic Sensor Modeling
         ======================================================================================== -->
    <q-separator spaced="lg" color="indigo-9" />

    <SectionTitle class="q-mt-xl">3. Sensor Model: Likelihood Field</SectionTitle>

    <TextBlock>
      El modelo de sensor define cómo calculamos la probabilidad
      <MathFormula :formula="'p(z_t | x_t, map)'" inline /> de observar un escaneo láser
      <MathFormula :formula="'z_t'" inline /> dada una pose hipotética
      <MathFormula :formula="'x_t'" inline /> y el mapa. AMCL soporta dos modelos:
      <strong>Beam Model</strong> (ray casting completo) y <strong>Likelihood Field</strong>
      (pre-computado, más rápido).
    </TextBlock>

    <!-- SENSOR MODEL COMPARISON -->
    <div class="comparison-grid q-my-lg">
      <div class="comparison-card beam">
        <div class="card-header">
          <q-icon name="straighten" size="lg" />
          <span>Beam Model</span>
        </div>
        <div class="card-pros">
          <div class="pros-title">✅ Ventajas</div>
          <ul>
            <li>Físicamente preciso (ray tracing real)</li>
            <li>Modela obstáculos dinámicos</li>
            <li>Mejor para mapas con objetos móviles</li>
          </ul>
        </div>
        <div class="card-cons">
          <div class="cons-title">❌ Desventajas</div>
          <ul>
            <li>Computacionalmente costoso (O(n²))</li>
            <li>Sensible a ruido del mapa</li>
            <li>Requiere ray casting en cada partícula</li>
          </ul>
        </div>
      </div>

      <div class="comparison-card likelihood">
        <div class="card-header">
          <q-icon name="blur_on" size="lg" />
          <span>Likelihood Field (Recomendado)</span>
        </div>
        <div class="card-pros">
          <div class="pros-title">✅ Ventajas</div>
          <ul>
            <li>Rápido: O(1) lookup en grid pre-computado</li>
            <li>Suave y robusto al ruido</li>
            <li>Mejor convergencia en la práctica</li>
          </ul>
        </div>
        <div class="card-cons">
          <div class="cons-title">❌ Desventajas</div>
          <ul>
            <li>No modela obstáculos dinámicos</li>
            <li>Requiere pre-procesamiento del mapa</li>
            <li>Menos preciso en teoría</li>
          </ul>
        </div>
      </div>

      <!-- LIKELIHOOD FIELD CODE PLACEHOLDER -->
      <AlertBlock type="info" title="Implementación Pendiente">
        <strong>TODO:</strong> Agregar implementación completa del Likelihood Field Model en C++
        (~500 líneas) incluyendo:
        <ul class="q-mt-sm">
          <li>Pre-computación del distance transform</li>
          <li>Cálculo de likelihood para cada rayo láser</li>
          <li>Manejo de z_hit, z_rand, z_max, z_short</li>
          <li>Optimizaciones con lookup tables</li>
        </ul>
      </AlertBlock>
    </div>

    <!-- ========================================================================================
         BLOCK D: AMCL PARAMETER ENCYCLOPEDIA
         Comprehensive documentation of all 60+ parameters
         ======================================================================================== -->
    <div class="content-block q-mt-xl">
      <SectionTitle class="q-mt-xl">4. Enciclopedia de Parámetros AMCL</SectionTitle>

      <TextBlock>
        AMCL tiene más de <strong>60 parámetros configurables</strong>. El tuning correcto es la
        diferencia entre un robot que se localiza perfectamente y uno que "salta" constantemente o
        se pierde en pasillos.
      </TextBlock>

      <!-- PARAMETER CATEGORIES -->
      <div class="param-categories q-my-lg">
        <div class="category-card filter">
          <div class="cat-icon">🎲</div>
          <div class="cat-title">Filter Parameters</div>
          <div class="cat-desc">Control de población de partículas y resampling</div>
          <div class="cat-count">12 parámetros</div>
        </div>

        <div class="category-card laser">
          <div class="cat-icon">📡</div>
          <div class="cat-title">Laser Model</div>
          <div class="cat-desc">Configuración del modelo de sensor</div>
          <div class="cat-count">15 parámetros</div>
        </div>

        <div class="category-card odom">
          <div class="cat-icon">🔄</div>
          <div class="cat-title">Odometry Model</div>
          <div class="cat-desc">Ruido del motion model (alphas)</div>
          <div class="cat-count">10 parámetros</div>
        </div>

        <div class="category-card init">
          <div class="cat-icon">🎯</div>
          <div class="cat-title">Initial Pose</div>
          <div class="cat-desc">Pose inicial y covarianza</div>
          <div class="cat-count">6 parámetros</div>
        </div>
      </div>

      <!-- COMPLETE AMCL PARAMETER TABLE -->
      <div class="param-table-container q-mt-lg">
        <div class="table-title">📊 Enciclopedia Completa de Parámetros AMCL (60+ parámetros)</div>
        <div class="table-wrapper">
          <table class="param-table">
            <thead>
              <tr>
                <th>Parámetro</th>
                <th>Tipo</th>
                <th>Default</th>
                <th>Rango</th>
                <th>Descripción Técnica</th>
                <th>Impacto</th>
              </tr>
            </thead>
            <tbody>
              <!-- FILTER PARAMETERS -->
              <tr class="category-row">
                <td colspan="6">🎲 Filter Parameters (Población de Partículas)</td>
              </tr>
              <tr>
                <td><code>min_particles</code></td>
                <td>int</td>
                <td>500</td>
                <td>100-10000</td>
                <td>
                  Mínimo número de partículas permitidas. KLD-Sampling nunca reducirá por debajo de
                  este valor.
                </td>
                <td>⚡ CPU, 🎯 Precision</td>
              </tr>
              <tr>
                <td><code>max_particles</code></td>
                <td>int</td>
                <td>5000</td>
                <td>500-50000</td>
                <td>
                  Máximo número de partículas. Limita el crecimiento adaptativo para evitar
                  explosión computacional.
                </td>
                <td>⚡⚡⚡ CPU, 🎯🎯 Precision</td>
              </tr>
              <tr>
                <td><code>kld_err</code></td>
                <td>double</td>
                <td>0.05</td>
                <td>0.01-0.1</td>
                <td>
                  Error máximo entre distribución real y estimada (Kullback-Leibler Divergence).
                  Valores menores = más partículas.
                </td>
                <td>🎯🎯 Precision, ⚡ CPU</td>
              </tr>
              <tr>
                <td><code>kld_z</code></td>
                <td>double</td>
                <td>0.99</td>
                <td>0.90-0.999</td>
                <td>
                  Cuantil superior de la distribución chi-cuadrado (nivel de confianza). 0.99 = 99%
                  de confianza.
                </td>
                <td>🎯 Precision</td>
              </tr>
              <tr>
                <td><code>update_min_d</code></td>
                <td>double</td>
                <td>0.2</td>
                <td>0.05-1.0</td>
                <td>
                  Distancia mínima (metros) que el robot debe moverse antes de realizar un update
                  del filtro.
                </td>
                <td>⚡⚡ CPU, 🎯 Precision</td>
              </tr>
              <tr>
                <td><code>update_min_a</code></td>
                <td>double</td>
                <td>0.5</td>
                <td>0.1-1.57</td>
                <td>
                  Ángulo mínimo (radianes) que el robot debe rotar antes de realizar un update. π/6
                  ≈ 0.52 rad ≈ 30°.
                </td>
                <td>⚡⚡ CPU, 🎯 Precision</td>
              </tr>
              <tr>
                <td><code>resample_interval</code></td>
                <td>int</td>
                <td>1</td>
                <td>1-10</td>
                <td>
                  Número de updates entre resampling. 1 = resamplear cada update, 2 = cada 2
                  updates.
                </td>
                <td>⚡ CPU, 🎯 Convergence</td>
              </tr>
              <tr>
                <td><code>transform_tolerance</code></td>
                <td>double</td>
                <td>1.0</td>
                <td>0.1-5.0</td>
                <td>
                  Tolerancia temporal (segundos) para transforms. Permite latencia en TF tree.
                </td>
                <td>⚠️ Stability</td>
              </tr>
              <tr>
                <td><code>recovery_alpha_slow</code></td>
                <td>double</td>
                <td>0.0</td>
                <td>0.0-0.5</td>
                <td>
                  Tasa de decaimiento exponencial para promedio lento de likelihood. 0.0 =
                  deshabilitado.
                </td>
                <td>🔄 Recovery</td>
              </tr>
              <tr>
                <td><code>recovery_alpha_fast</code></td>
                <td>double</td>
                <td>0.0</td>
                <td>0.0-0.5</td>
                <td>
                  Tasa de decaimiento exponencial para promedio rápido. Detecta kidnapped robot
                  cuando fast &lt; slow.
                </td>
                <td>🔄 Recovery</td>
              </tr>
              <tr>
                <td><code>initial_pose_x</code></td>
                <td>double</td>
                <td>0.0</td>
                <td>-∞ to +∞</td>
                <td>Posición inicial X en el mapa (metros).</td>
                <td>🎯 Initialization</td>
              </tr>
              <tr>
                <td><code>initial_pose_y</code></td>
                <td>double</td>
                <td>0.0</td>
                <td>-∞ to +∞</td>
                <td>Posición inicial Y en el mapa (metros).</td>
                <td>🎯 Initialization</td>
              </tr>
              <tr>
                <td><code>initial_pose_a</code></td>
                <td>double</td>
                <td>0.0</td>
                <td>-π to +π</td>
                <td>Orientación inicial (radianes). 0 = apuntando hacia +X.</td>
                <td>🎯 Initialization</td>
              </tr>

              <!-- LASER MODEL PARAMETERS -->
              <tr class="category-row">
                <td colspan="6">📡 Laser Model Parameters (Modelo de Sensor)</td>
              </tr>
              <tr>
                <td><code>laser_model_type</code></td>
                <td>string</td>
                <td>likelihood_field</td>
                <td>beam, likelihood_field, likelihood_field_prob</td>
                <td>
                  Tipo de modelo de sensor. likelihood_field es más rápido, beam es más preciso.
                </td>
                <td>⚡⚡ CPU, 🎯 Accuracy</td>
              </tr>
              <tr>
                <td><code>laser_max_beams</code></td>
                <td>int</td>
                <td>30</td>
                <td>10-720</td>
                <td>
                  Número de rayos láser a usar (muestreo del scan completo). Menos rayos = menos
                  CPU.
                </td>
                <td>⚡⚡⚡ CPU, 🎯 Precision</td>
              </tr>
              <tr>
                <td><code>laser_z_hit</code></td>
                <td>double</td>
                <td>0.5</td>
                <td>0.0-1.0</td>
                <td>
                  Peso del componente de medición correcta (hit). Suma de z_hit + z_rand + z_max +
                  z_short = 1.0.
                </td>
                <td>🎯🎯 Sensor Trust</td>
              </tr>
              <tr>
                <td><code>laser_z_rand</code></td>
                <td>double</td>
                <td>0.5</td>
                <td>0.0-1.0</td>
                <td>
                  Peso del componente de medición aleatoria (ruido uniforme). Para sensores
                  ruidosos.
                </td>
                <td>🎯 Robustness</td>
              </tr>
              <tr>
                <td><code>laser_z_max</code></td>
                <td>double</td>
                <td>0.05</td>
                <td>0.0-1.0</td>
                <td>
                  Peso del componente de max-range (sensor saturado). Para detectar objetos fuera de
                  rango.
                </td>
                <td>🎯 Edge Cases</td>
              </tr>
              <tr>
                <td><code>laser_z_short</code></td>
                <td>double</td>
                <td>0.05</td>
                <td>0.0-1.0</td>
                <td>
                  Peso del componente de medición corta (obstáculos inesperados). Para objetos
                  dinámicos.
                </td>
                <td>🎯 Dynamic Obstacles</td>
              </tr>
              <tr>
                <td><code>laser_sigma_hit</code></td>
                <td>double</td>
                <td>0.2</td>
                <td>0.01-1.0</td>
                <td>
                  Desviación estándar del componente hit (metros). Modela incertidumbre del sensor.
                </td>
                <td>🎯🎯 Sensor Noise</td>
              </tr>
              <tr>
                <td><code>laser_lambda_short</code></td>
                <td>double</td>
                <td>0.1</td>
                <td>0.01-1.0</td>
                <td>Parámetro exponencial para distribución de mediciones cortas.</td>
                <td>🎯 Dynamic Handling</td>
              </tr>
              <tr>
                <td><code>laser_likelihood_max_dist</code></td>
                <td>double</td>
                <td>2.0</td>
                <td>0.5-10.0</td>
                <td>
                  Distancia máxima (metros) para considerar en likelihood field. Más grande = más
                  CPU.
                </td>
                <td>⚡ CPU, 🎯 Range</td>
              </tr>
              <tr>
                <td><code>laser_max_range</code></td>
                <td>double</td>
                <td>100.0</td>
                <td>1.0-1000.0</td>
                <td>Rango máximo del sensor (metros). Mediciones mayores se ignoran.</td>
                <td>🎯 Sensor Spec</td>
              </tr>
              <tr>
                <td><code>laser_min_range</code></td>
                <td>double</td>
                <td>0.0</td>
                <td>0.0-1.0</td>
                <td>
                  Rango mínimo del sensor (metros). Mediciones menores se ignoran (blind spot).
                </td>
                <td>🎯 Sensor Spec</td>
              </tr>

              <!-- ODOMETRY MODEL PARAMETERS -->
              <tr class="category-row">
                <td colspan="6">🔄 Odometry Model Parameters (Modelo de Movimiento)</td>
              </tr>
              <tr>
                <td><code>odom_model_type</code></td>
                <td>string</td>
                <td>diff</td>
                <td>diff, omni, diff-corrected, omni-corrected</td>
                <td>
                  Tipo de modelo cinemático. diff = differential drive, omni = omnidirectional.
                </td>
                <td>🎯🎯 Motion Model</td>
              </tr>
              <tr>
                <td><code>odom_alpha1</code></td>
                <td>double</td>
                <td>0.2</td>
                <td>0.0-2.0</td>
                <td>Error de rotación causado por rotación (rot→rot). Ruido en giros.</td>
                <td>🎯🎯🎯 Motion Noise</td>
              </tr>
              <tr>
                <td><code>odom_alpha2</code></td>
                <td>double</td>
                <td>0.2</td>
                <td>0.0-2.0</td>
                <td>
                  Error de rotación causado por traslación (trans→rot). Deriva angular al avanzar
                  recto.
                </td>
                <td>🎯🎯 Motion Noise</td>
              </tr>
              <tr>
                <td><code>odom_alpha3</code></td>
                <td>double</td>
                <td>0.2</td>
                <td>0.0-2.0</td>
                <td>Error de traslación causado por traslación (trans→trans). Slip de ruedas.</td>
                <td>🎯🎯🎯 Motion Noise</td>
              </tr>
              <tr>
                <td><code>odom_alpha4</code></td>
                <td>double</td>
                <td>0.2</td>
                <td>0.0-2.0</td>
                <td>
                  Error de traslación causado por rotación (rot→trans). Movimiento lateral al girar.
                </td>
                <td>🎯🎯 Motion Noise</td>
              </tr>
              <tr>
                <td><code>odom_alpha5</code></td>
                <td>double</td>
                <td>0.2</td>
                <td>0.0-2.0</td>
                <td>Error de traslación (solo para omni). Ruido en movimiento lateral.</td>
                <td>🎯 Omni Only</td>
              </tr>

              <!-- ADVANCED PARAMETERS -->
              <tr class="category-row">
                <td colspan="6">⚙️ Advanced Parameters (Configuración Avanzada)</td>
              </tr>
              <tr>
                <td><code>save_pose_rate</code></td>
                <td>double</td>
                <td>0.5</td>
                <td>0.1-10.0</td>
                <td>Frecuencia (Hz) para guardar pose estimada. Para logging/debugging.</td>
                <td>💾 Logging</td>
              </tr>
              <tr>
                <td><code>use_map_topic</code></td>
                <td>bool</td>
                <td>false</td>
                <td>true, false</td>
                <td>Si true, suscribe a topic /map. Si false, usa servicio static_map.</td>
                <td>🗺️ Map Source</td>
              </tr>
              <tr>
                <td><code>first_map_only</code></td>
                <td>bool</td>
                <td>false</td>
                <td>true, false</td>
                <td>Si true, ignora actualizaciones del mapa después de la primera.</td>
                <td>🗺️ Map Updates</td>
              </tr>
              <tr>
                <td><code>gui_publish_rate</code></td>
                <td>double</td>
                <td>-1.0</td>
                <td>-1.0-50.0</td>
                <td>Frecuencia (Hz) para publicar pose para GUI. -1.0 = máxima velocidad.</td>
                <td>📊 Visualization</td>
              </tr>
              <tr>
                <td><code>selective_resampling</code></td>
                <td>bool</td>
                <td>false</td>
                <td>true, false</td>
                <td>Si true, solo resamplea cuando la varianza de pesos es alta. Ahorra CPU.</td>
                <td>⚡ CPU Optimization</td>
              </tr>
              <tr>
                <td><code>tf_broadcast</code></td>
                <td>bool</td>
                <td>true</td>
                <td>true, false</td>
                <td>Si true, publica transform map→odom. Desactivar solo para debugging.</td>
                <td>🔗 TF Tree</td>
              </tr>
              <tr>
                <td><code>do_beamskip</code></td>
                <td>bool</td>
                <td>false</td>
                <td>true, false</td>
                <td>Si true, ignora scans cuando el robot está quieto. Optimización agresiva.</td>
                <td>⚡⚡ CPU Optimization</td>
              </tr>
              <tr>
                <td><code>beam_skip_distance</code></td>
                <td>double</td>
                <td>0.5</td>
                <td>0.1-2.0</td>
                <td>
                  Distancia (metros) para considerar que el robot está quieto (si do_beamskip=true).
                </td>
                <td>⚡ Beamskip Threshold</td>
              </tr>
              <tr>
                <td><code>beam_skip_threshold</code></td>
                <td>double</td>
                <td>0.3</td>
                <td>0.0-1.0</td>
                <td>
                  Fracción de rayos que deben cambiar para procesar scan (si do_beamskip=true).
                </td>
                <td>⚡ Beamskip Sensitivity</td>
              </tr>
              <tr>
                <td><code>beam_skip_error_threshold</code></td>
                <td>double</td>
                <td>0.9</td>
                <td>0.0-1.0</td>
                <td>Threshold de error para activar beamskip. Valores altos = más agresivo.</td>
                <td>⚡ Beamskip Aggressiveness</td>
              </tr>
            </tbody>
          </table>
        </div>

        <div class="table-footer">
          <div class="footer-note">
            <strong>💡 Nota de Calibración:</strong> Los valores por defecto funcionan bien para la
            mayoría de casos, pero debes calibrar <code>odom_alpha1-4</code> según tu tipo de suelo
            (carpet vs tile vs outdoor) y <code>laser_z_hit/z_rand</code> según la calidad de tu
            sensor. Usa <code>ros2 run amcl amcl --ros-args --params-file config.yaml</code> para
            cargar configuración.
          </div>
        </div>
      </div>
    </div>

    <!-- ========================================================================================
         BLOCK E: TF TREE & SYNCHRONIZATION
         Transform tree visualization and message_filters
         ======================================================================================== -->
    <q-separator spaced="lg" color="indigo-9" />

    <div class="content-block">
      <SectionTitle class="q-mt-xl">5. TF Tree y Sincronización Temporal</SectionTitle>

      <TextBlock>
        La única responsabilidad de AMCL es publicar la transformación
        <code>map → odom</code> que corrige el drift acumulado de la odometría. Esta transformada
        conecta el marco de referencia global (mapa) con el marco local (odometría).
      </TextBlock>

      <!-- COMPLETE TF TREE DIAGRAM -->
      <div class="tf-tree-container q-my-lg">
        <div class="tree-header">
          <q-icon name="account_tree" />
          <span>Árbol de Transformadas (TF Tree)</span>
        </div>
        <div class="tree-content">
          <div class="tf-tree-viz">
            <div class="tree-canvas">
              <svg viewBox="0 0 700 450" class="tf-diagram">
                <defs>
                  <marker
                    id="arrowhead"
                    markerWidth="10"
                    markerHeight="7"
                    refX="9"
                    refY="3.5"
                    orient="auto"
                  >
                    <polygon points="0 0, 10 3.5, 0 7" fill="#3b82f6" />
                  </marker>
                  <filter id="glow">
                    <feGaussianBlur stdDeviation="3" result="coloredBlur" />
                    <feMerge>
                      <feMergeNode in="coloredBlur" />
                      <feMergeNode in="SourceGraphic" />
                    </feMerge>
                  </filter>
                </defs>

                <!-- MAP frame (root) -->
                <g class="tf-frame map-frame">
                  <rect
                    x="275"
                    y="50"
                    width="150"
                    height="80"
                    rx="12"
                    fill="#1e293b"
                    stroke="#22c55e"
                    stroke-width="3"
                  />
                  <text
                    x="350"
                    y="85"
                    text-anchor="middle"
                    fill="#22c55e"
                    font-weight="700"
                    font-size="20"
                  >
                    /map
                  </text>
                  <text x="350" y="105" text-anchor="middle" fill="#94a3b8" font-size="12">
                    Global Fixed Frame
                  </text>
                  <text x="350" y="120" text-anchor="middle" fill="#64748b" font-size="10">
                    Published by: map_server
                  </text>
                </g>

                <!-- ODOM frame -->
                <g class="tf-frame odom-frame">
                  <rect
                    x="275"
                    y="180"
                    width="150"
                    height="80"
                    rx="12"
                    fill="#1e293b"
                    stroke="#3b82f6"
                    stroke-width="3"
                  />
                  <text
                    x="350"
                    y="215"
                    text-anchor="middle"
                    fill="#3b82f6"
                    font-weight="700"
                    font-size="20"
                  >
                    /odom
                  </text>
                  <text x="350" y="235" text-anchor="middle" fill="#94a3b8" font-size="12">
                    Odometry Frame
                  </text>
                  <text x="350" y="250" text-anchor="middle" fill="#64748b" font-size="10">
                    Published by: diff_drive
                  </text>
                </g>

                <!-- BASE_LINK frame -->
                <g class="tf-frame base-frame">
                  <rect
                    x="275"
                    y="310"
                    width="150"
                    height="80"
                    rx="12"
                    fill="#1e293b"
                    stroke="#a855f7"
                    stroke-width="3"
                  />
                  <text
                    x="350"
                    y="345"
                    text-anchor="middle"
                    fill="#a855f7"
                    font-weight="700"
                    font-size="20"
                  >
                    /base_link
                  </text>
                  <text x="350" y="365" text-anchor="middle" fill="#94a3b8" font-size="12">
                    Robot Center
                  </text>
                  <text x="350" y="380" text-anchor="middle" fill="#64748b" font-size="10">
                    Published by: robot_state_publisher
                  </text>
                </g>

                <!-- LASER_LINK frame -->
                <g class="tf-frame laser-frame">
                  <rect
                    x="500"
                    y="310"
                    width="150"
                    height="80"
                    rx="12"
                    fill="#1e293b"
                    stroke="#f59e0b"
                    stroke-width="3"
                  />
                  <text
                    x="575"
                    y="345"
                    text-anchor="middle"
                    fill="#f59e0b"
                    font-weight="700"
                    font-size="20"
                  >
                    /laser_link
                  </text>
                  <text x="575" y="365" text-anchor="middle" fill="#94a3b8" font-size="12">
                    Lidar Sensor
                  </text>
                  <text x="575" y="380" text-anchor="middle" fill="#64748b" font-size="10">
                    Published by: static_transform
                  </text>
                </g>

                <!-- IMU_LINK frame (optional) -->
                <g class="tf-frame imu-frame">
                  <rect
                    x="50"
                    y="310"
                    width="150"
                    height="80"
                    rx="12"
                    fill="#1e293b"
                    stroke="#ec4899"
                    stroke-width="3"
                  />
                  <text
                    x="125"
                    y="345"
                    text-anchor="middle"
                    fill="#ec4899"
                    font-weight="700"
                    font-size="20"
                  >
                    /imu_link
                  </text>
                  <text x="125" y="365" text-anchor="middle" fill="#94a3b8" font-size="12">
                    IMU Sensor
                  </text>
                  <text x="125" y="380" text-anchor="middle" fill="#64748b" font-size="10">
                    Published by: static_transform
                  </text>
                </g>

                <!-- Transforms (arrows with labels) -->
                <!-- map → odom (AMCL) -->
                <line
                  x1="350"
                  y1="130"
                  x2="350"
                  y2="180"
                  stroke="#22c55e"
                  stroke-width="3"
                  marker-end="url(#arrowhead)"
                />
                <rect
                  x="360"
                  y="145"
                  width="100"
                  height="30"
                  rx="6"
                  fill="#1e293b"
                  stroke="#22c55e"
                  stroke-width="2"
                />
                <text
                  x="410"
                  y="163"
                  text-anchor="middle"
                  fill="#22c55e"
                  font-size="12"
                  font-weight="700"
                >
                  AMCL
                </text>
                <text x="410" y="173" text-anchor="middle" fill="#64748b" font-size="9">~2 Hz</text>

                <!-- odom → base_link (diff_drive) -->
                <line
                  x1="350"
                  y1="260"
                  x2="350"
                  y2="310"
                  stroke="#3b82f6"
                  stroke-width="3"
                  marker-end="url(#arrowhead)"
                />
                <rect
                  x="360"
                  y="275"
                  width="120"
                  height="30"
                  rx="6"
                  fill="#1e293b"
                  stroke="#3b82f6"
                  stroke-width="2"
                />
                <text
                  x="420"
                  y="293"
                  text-anchor="middle"
                  fill="#3b82f6"
                  font-size="12"
                  font-weight="700"
                >
                  diff_drive
                </text>
                <text x="420" y="303" text-anchor="middle" fill="#64748b" font-size="9">
                  ~50 Hz
                </text>

                <!-- base_link → laser_link (static) -->
                <line
                  x1="425"
                  y1="350"
                  x2="500"
                  y2="350"
                  stroke="#a855f7"
                  stroke-width="3"
                  marker-end="url(#arrowhead)"
                />
                <rect
                  x="440"
                  y="330"
                  width="80"
                  height="30"
                  rx="6"
                  fill="#1e293b"
                  stroke="#a855f7"
                  stroke-width="2"
                />
                <text
                  x="480"
                  y="348"
                  text-anchor="middle"
                  fill="#a855f7"
                  font-size="12"
                  font-weight="700"
                >
                  static_tf
                </text>
                <text x="480" y="358" text-anchor="middle" fill="#64748b" font-size="9">∞ Hz</text>

                <!-- base_link → imu_link (static) -->
                <line
                  x1="275"
                  y1="350"
                  x2="200"
                  y2="350"
                  stroke="#a855f7"
                  stroke-width="3"
                  marker-end="url(#arrowhead)"
                />
                <rect
                  x="210"
                  y="330"
                  width="80"
                  height="30"
                  rx="6"
                  fill="#1e293b"
                  stroke="#a855f7"
                  stroke-width="2"
                />
                <text
                  x="250"
                  y="348"
                  text-anchor="middle"
                  fill="#a855f7"
                  font-size="12"
                  font-weight="700"
                >
                  static_tf
                </text>
                <text x="250" y="358" text-anchor="middle" fill="#64748b" font-size="9">∞ Hz</text>

                <!-- Legend annotations -->
                <text x="50" y="30" fill="#cbd5e1" font-size="14" font-weight="600">
                  Transform Chain:
                </text>
                <text x="50" y="45" fill="#94a3b8" font-size="11">
                  /map → /odom → /base_link → /laser_link
                </text>
              </svg>
            </div>

            <div class="tree-legend">
              <div class="legend-title">Leyenda de Frames</div>
              <div class="legend-grid">
                <div class="legend-item">
                  <div class="legend-color" style="background: #22c55e"></div>
                  <div class="legend-info">
                    <span class="legend-name">/map</span>
                    <span class="legend-desc">Frame global fijo (no deriva)</span>
                  </div>
                </div>
                <div class="legend-item">
                  <div class="legend-color" style="background: #3b82f6"></div>
                  <div class="legend-info">
                    <span class="legend-name">/odom</span>
                    <span class="legend-desc">Frame de odometría (deriva con el tiempo)</span>
                  </div>
                </div>
                <div class="legend-item">
                  <div class="legend-color" style="background: #a855f7"></div>
                  <div class="legend-info">
                    <span class="legend-name">/base_link</span>
                    <span class="legend-desc">Centro del robot</span>
                  </div>
                </div>
                <div class="legend-item">
                  <div class="legend-color" style="background: #f59e0b"></div>
                  <div class="legend-info">
                    <span class="legend-name">/laser_link</span>
                    <span class="legend-desc">Sensor Lidar</span>
                  </div>
                </div>
                <div class="legend-item">
                  <div class="legend-color" style="background: #ec4899"></div>
                  <div class="legend-info">
                    <span class="legend-name">/imu_link</span>
                    <span class="legend-desc">Sensor IMU (opcional)</span>
                  </div>
                </div>
              </div>
            </div>
          </div>

          <AltertBlock type="info" title="Responsabilidad de AMCL" class="q-mt-lg">
            <strong>AMCL solo publica la transformación /map → /odom.</strong> Esta transform
            corrige el drift acumulado de la odometría, permitiendo que el robot se localice en el
            mapa global. Las demás transformaciones son publicadas por otros nodos:
            <ul class="q-mt-sm">
              <li>
                <code>diff_drive_controller</code>: Publica /odom → /base_link basándose en encoders
                de rueda
              </li>
              <li><code>robot_state_publisher</code>: Publica transforms estáticas del URDF</li>
              <li>
                <code>static_transform_publisher</code>: Publica /base_link → /laser_link y
                /base_link → /imu_link
              </li>
            </ul>
          </AltertBlock>
        </div>
      </div>

      <!-- MESSAGE FILTERS CODE PLACEHOLDER -->
      <AlertBlock type="warning" title="Código de Sincronización Pendiente">
        <strong>TODO:</strong> Agregar implementación de message_filters para sincronizar LaserScan
        con TF:
        <ul class="q-mt-sm">
          <li>ApproximateTime synchronizer</li>
          <li>Manejo de timestamps</li>
          <li>Buffer de TF con extrapolación</li>
          <li>Debugging de problemas de sincronización</li>
        </ul>
      </AlertBlock>

      <!-- ========================================================================================
         BLOCK F: KIDNAPPED ROBOT PROBLEM
         Global localization and recovery
         ======================================================================================== -->
      <q-separator spaced="lg" color="indigo-9" />

      <SectionTitle class="q-mt-xl">6. The Kidnapped Robot Problem</SectionTitle>

      <TextBlock>
        Si mueves el robot manualmente (teleportación), las partículas divergen completamente porque
        están concentradas en la posición anterior. AMCL necesita
        <strong>relocalización global</strong> para recuperarse.
      </TextBlock>

      <!-- RECOVERY METHODS -->
      <div class="recovery-grid q-my-lg">
        <div class="recovery-method gui">
          <div class="method-header">
            <q-icon name="mouse" />
            <span>Método 1: RViz GUI</span>
          </div>
          <div class="method-body">
            Usa la herramienta <code>2D Pose Estimate</code> en RViz para dar una estimación inicial
            manual haciendo clic en el mapa.
          </div>
        </div>

        <div class="recovery-method service">
          <div class="method-header">
            <q-icon name="api" />
            <span>Método 2: Service Call</span>
          </div>
          <div class="method-body">
            Llama al servicio <code>/reinitialize_global_localization</code> para dispersar
            partículas uniformemente por todo el mapa.
          </div>
        </div>

        <div class="recovery-method auto">
          <div class="method-header">
            <q-icon name="autorenew" />
            <span>Método 3: Auto-Recovery</span>
          </div>
          <div class="method-body">
            Configura <code>recovery_alpha_slow</code> y <code>recovery_alpha_fast</code> para
            detección automática de secuestro.
          </div>
        </div>
      </div>

      <AlertBlock type="warning" title="Implementación Pendiente">
        <strong>TODO:</strong> Agregar nodo custom de auto-recovery con:
        <ul class="q-mt-sm">
          <li>Detección de divergencia de partículas</li>
          <li>Trigger automático de global localization</li>
          <li>Estrategias de búsqueda inteligente</li>
          <li>Código completo en C++ y Python</li>
        </ul>
      </AlertBlock>

      <!-- ========================================================================================
         BLOCK G: INTERACTIVE TOOLS & SIMULATORS
         Real-time parameter tuning and visualization
         ======================================================================================== -->
      <q-separator spaced="lg" color="indigo-9" />

      <SectionTitle class="q-mt-xl">7. Herramientas Interactivas</SectionTitle>

      <TextBlock>
        Experimenta con los parámetros de AMCL en tiempo real para entender su impacto en la
        localización.
      </TextBlock>

      <!-- PARTICLE FILTER SIMULATOR -->
      <div class="tool-wrapper q-my-lg">
        <div class="tool-header">
          <q-icon name="tune" />
          <div class="tool-title">Particle Filter Simulator (Interactive)</div>
          <div class="tool-subtitle">
            Ajusta parámetros y observa el comportamiento en tiempo real
          </div>
        </div>

        <div class="particle-lab">
          <!-- LEFT: CONTROLS -->
          <div class="lab-controls">
            <div class="control-section">
              <div class="section-label">🎲 Filter Parameters</div>

              <div class="control-group">
                <div class="c-label">
                  <span>Number of Particles</span>
                  <q-icon name="info" size="xs" color="blue-4">
                    <q-tooltip>Más partículas = mayor precisión pero más CPU</q-tooltip>
                  </q-icon>
                </div>
                <q-slider
                  v-model="numParticles"
                  :min="100"
                  :max="5000"
                  :step="100"
                  label
                  label-always
                  color="blue-4"
                  class="q-mt-sm"
                />
                <div class="c-value">N = {{ numParticles }}</div>
              </div>

              <div class="control-group">
                <div class="c-label">
                  <span>KLD Error Threshold</span>
                  <q-icon name="info" size="xs" color="purple-4">
                    <q-tooltip>Menor valor = más partículas adaptativas</q-tooltip>
                  </q-icon>
                </div>
                <q-slider
                  v-model="kldError"
                  :min="0.01"
                  :max="0.1"
                  :step="0.01"
                  label
                  label-always
                  color="purple-4"
                  class="q-mt-sm"
                />
                <div class="c-value">ε = {{ kldError.toFixed(2) }}</div>
              </div>
            </div>

            <div class="control-section">
              <div class="section-label">🔄 Odometry Noise</div>

              <div class="control-group">
                <div class="c-label">
                  <span>Alpha 1 (rot→rot)</span>
                  <q-icon name="info" size="xs" color="orange-4">
                    <q-tooltip>Error de rotación causado por rotación</q-tooltip>
                  </q-icon>
                </div>
                <q-slider
                  v-model="alpha1"
                  :min="0"
                  :max="1"
                  :step="0.05"
                  label
                  label-always
                  color="orange-4"
                  class="q-mt-sm"
                />
                <div class="c-value">α₁ = {{ alpha1.toFixed(2) }}</div>
              </div>

              <div class="control-group">
                <div class="c-label">
                  <span>Alpha 3 (trans→trans)</span>
                  <q-icon name="info" size="xs" color="orange-4">
                    <q-tooltip>Error de traslación (slip de ruedas)</q-tooltip>
                  </q-icon>
                </div>
                <q-slider
                  v-model="alpha3"
                  :min="0"
                  :max="1"
                  :step="0.05"
                  label
                  label-always
                  color="orange-4"
                  class="q-mt-sm"
                />
                <div class="c-value">α₃ = {{ alpha3.toFixed(2) }}</div>
              </div>
            </div>

            <div class="control-section">
              <div class="section-label">📡 Sensor Model</div>

              <div class="control-group">
                <div class="c-label">
                  <span>Sensor Noise (σ)</span>
                  <q-icon name="info" size="xs" color="green-4">
                    <q-tooltip>Desviación estándar del Lidar</q-tooltip>
                  </q-icon>
                </div>
                <q-slider
                  v-model="sensorNoise"
                  :min="0"
                  :max="0.5"
                  :step="0.01"
                  label
                  label-always
                  color="green-4"
                  class="q-mt-sm"
                />
                <div class="c-value">σ = {{ sensorNoise.toFixed(2) }}m</div>
              </div>

              <div class="control-group">
                <div class="c-label">
                  <span>z_hit (Confianza)</span>
                  <q-icon name="info" size="xs" color="green-4">
                    <q-tooltip>Peso de mediciones correctas</q-tooltip>
                  </q-icon>
                </div>
                <q-slider
                  v-model="zHit"
                  :min="0"
                  :max="1"
                  :step="0.05"
                  label
                  label-always
                  color="green-4"
                  class="q-mt-sm"
                />
                <div class="c-value">z_hit = {{ zHit.toFixed(2) }}</div>
              </div>
            </div>

            <!-- PRESETS -->
            <div class="presets-section">
              <div class="section-label">⚙️ Presets</div>
              <div class="preset-buttons">
                <q-btn
                  flat
                  dense
                  label="Indoor"
                  color="blue"
                  @click="applyPreset('indoor')"
                  class="preset-btn"
                />
                <q-btn
                  flat
                  dense
                  label="Outdoor"
                  color="green"
                  @click="applyPreset('outdoor')"
                  class="preset-btn"
                />
                <q-btn
                  flat
                  dense
                  label="Warehouse"
                  color="orange"
                  @click="applyPreset('warehouse')"
                  class="preset-btn"
                />
                <q-btn
                  flat
                  dense
                  label="Reset"
                  color="grey"
                  @click="applyPreset('default')"
                  class="preset-btn"
                />
              </div>
            </div>
          </div>

          <!-- RIGHT: VISUALIZATION -->
          <div class="lab-viz">
            <div class="viz-header">
              <q-icon name="visibility" />
              <span>Particle Distribution Visualization</span>
            </div>

            <div class="particle-plot">
              <svg viewBox="0 0 400 400" class="particle-canvas">
                <defs>
                  <radialGradient id="particleGrad">
                    <stop offset="0%" stop-color="#ef4444" stop-opacity="0.8" />
                    <stop offset="100%" stop-color="#ef4444" stop-opacity="0" />
                  </radialGradient>
                  <filter id="particleGlow">
                    <feGaussianBlur stdDeviation="2" result="coloredBlur" />
                    <feMerge>
                      <feMergeNode in="coloredBlur" />
                      <feMergeNode in="SourceGraphic" />
                    </feMerge>
                  </filter>
                </defs>

                <!-- Grid background -->
                <g class="grid" opacity="0.1">
                  <line
                    v-for="i in 8"
                    :key="'v' + i"
                    :x1="i * 50"
                    y1="0"
                    :x2="i * 50"
                    y2="400"
                    stroke="#94a3b8"
                    stroke-width="1"
                  />
                  <line
                    v-for="i in 8"
                    :key="'h' + i"
                    x1="0"
                    :y1="i * 50"
                    x2="400"
                    :y2="i * 50"
                    stroke="#94a3b8"
                    stroke-width="1"
                  />
                </g>

                <!-- Uncertainty ellipse -->
                <ellipse
                  :cx="200"
                  :cy="200"
                  :rx="50 + alpha1 * 100"
                  :ry="50 + alpha3 * 100"
                  fill="none"
                  stroke="#3b82f6"
                  stroke-width="2"
                  opacity="0.3"
                  stroke-dasharray="5,5"
                />

                <!-- Particle cloud -->
                <g class="particles">
                  <circle
                    v-for="i in Math.min(numParticles, 200)"
                    :key="i"
                    :cx="
                      200 +
                      Math.cos((i / numParticles) * Math.PI * 2) *
                        (50 + Math.random() * alpha1 * 100)
                    "
                    :cy="
                      200 +
                      Math.sin((i / numParticles) * Math.PI * 2) *
                        (50 + Math.random() * alpha3 * 100)
                    "
                    :r="Math.max(1, 3 - numParticles / 2000)"
                    fill="url(#particleGrad)"
                    opacity="0.6"
                  />
                </g>

                <!-- Robot ground truth -->
                <g class="robot">
                  <circle cx="200" cy="200" r="8" fill="#10b981" filter="url(#particleGlow)" />
                  <circle
                    cx="200"
                    cy="200"
                    r="15"
                    fill="none"
                    stroke="#10b981"
                    stroke-width="2"
                    opacity="0.5"
                  />
                  <line x1="200" y1="200" x2="220" y2="200" stroke="#10b981" stroke-width="3" />
                </g>

                <!-- Sensor beam -->
                <line
                  x1="200"
                  y1="200"
                  :x2="200 + 150 * Math.cos(sensorBeamAngle)"
                  :y2="200 + 150 * Math.sin(sensorBeamAngle)"
                  stroke="#22d3ee"
                  stroke-width="2"
                  opacity="0.4"
                  class="sensor-beam"
                />
              </svg>

              <!-- Metrics -->
              <div class="plot-stats">
                <div class="stat-item">
                  <div class="stat-label">Convergence</div>
                  <div class="stat-value" :style="{ color: convergenceColor }">
                    {{ convergence.toFixed(1) }}%
                  </div>
                </div>
                <div class="stat-item">
                  <div class="stat-label">Uncertainty</div>
                  <div class="stat-value">{{ uncertainty.toFixed(0) }}cm</div>
                </div>
                <div class="stat-item">
                  <div class="stat-label">CPU Load</div>
                  <div class="stat-value" :style="{ color: cpuColor }">
                    {{ cpuLoad.toFixed(0) }}%
                  </div>
                </div>
              </div>
            </div>

            <!-- Insights -->
            <div class="insights-panel">
              <div class="insight-title">💡 Insights</div>
              <div class="insight-text">
                {{ currentInsight }}
              </div>
            </div>
          </div>
        </div>
      </div>

      <!-- ========================================================================================
         BLOCK H: PERFORMANCE OPTIMIZATION
         CPU profiling and benchmarking
         ======================================================================================== -->
      <q-separator spaced="lg" color="indigo-9" />

      <SectionTitle class="q-mt-xl">8. Optimización de Performance</SectionTitle>

      <TextBlock>
        AMCL puede consumir CPU significativa con muchas partículas. Aprende a optimizar sin
        sacrificar precisión.
      </TextBlock>

      <!-- OPTIMIZATION TIPS -->
      <div class="optimization-grid q-my-lg">
        <div class="opt-card">
          <div class="opt-icon">⚡</div>
          <div class="opt-title">Reduce Particle Count</div>
          <div class="opt-desc">
            Usa <code>min_particles: 500</code> y <code>max_particles: 2000</code> para mapas
            pequeños. KLD ajustará dinámicamente.
          </div>
        </div>

        <div class="opt-card">
          <div class="opt-icon">🎯</div>
          <div class="opt-title">Increase Update Thresholds</div>
          <div class="opt-desc">
            Configura <code>update_min_d: 0.25</code> y <code>update_min_a: 0.5</code> para evitar
            updates cuando el robot está casi quieto.
          </div>
        </div>

        <div class="opt-card">
          <div class="opt-icon">📊</div>
          <div class="opt-title">Reduce Laser Beams</div>
          <div class="opt-desc">
            Usa <code>laser_max_beams: 30</code> en lugar de 720. Muestrea solo algunos rayos del
            escaneo completo.
          </div>
        </div>

        <div class="opt-card">
          <div class="opt-icon">🔄</div>
          <div class="opt-title">Adjust Resample Interval</div>
          <div class="opt-desc">
            Configura <code>resample_interval: 2</code> para resamplear cada 2 updates en lugar de
            cada uno.
          </div>
        </div>
      </div>

      <AlertBlock type="warning" title="Benchmarking Pendiente">
        <strong>TODO:</strong> Agregar herramientas de profiling:
        <ul class="q-mt-sm">
          <li>Código para medir CPU usage de AMCL</li>
          <li>Comparación de tiempos con diferentes configuraciones</li>
          <li>Gráficas de performance vs accuracy</li>
          <li>Comparación: AMCL vs Cartographer vs SLAM Toolbox</li>
        </ul>
      </AlertBlock>

      <AlertBlock type="success" title="Checklist de Implementación" class="q-mt-lg">
        <strong>Antes de deployar AMCL en producción:</strong>
        <ul class="q-mt-sm">
          <li>✅ Verifica que el mapa esté bien construido (sin artefactos)</li>
          <li>✅ Calibra la odometría del robot (wheel diameter, baseline)</li>
          <li>✅ Ajusta alphas basándote en el tipo de suelo (carpet vs tile)</li>
          <li>✅ Prueba global localization en múltiples ubicaciones</li>
          <li>✅ Mide CPU usage y ajusta parámetros si es necesario</li>
          <li>✅ Implementa recovery behaviors para kidnapped robot</li>
          <li>✅ Documenta tu configuración final en YAML</li>
        </ul>
      </AlertBlock>

      <!-- ========================================================================================
         SECTION 7: CASE STUDY - AMCL ROBOT CONFIGURATION
         Real-world implementation example
         ======================================================================================== -->
      <q-separator spaced="lg" color="indigo-9" />

      <SectionTitle class="q-mt-xl">9. Case Study: Robot Autónomo con AMCL</SectionTitle>
      <TextBlock>
        Implementación completa de AMCL en un robot móvil industrial. Este sistema combina odometría
        de ruedas, Lidar 2D, y AMCL para localización robusta en almacenes.
      </TextBlock>

      <!-- ROBOT CONFIGURATION VIZ -->
      <div class="sensor-suite-viz q-my-lg">
        <div class="suite-header">
          <q-icon name="precision_manufacturing" size="xl" color="amber" />
          <div class="suite-title">Configuración del Sistema</div>
        </div>
        <div class="suite-grid">
          <div class="sensor-card lidar-card">
            <q-icon name="radar" size="lg" />
            <div class="sensor-name">Lidar 2D</div>
            <div class="sensor-spec">270° FOV, 30m range</div>
            <div class="sensor-freq">20 Hz</div>
          </div>
          <div class="sensor-card encoder-card">
            <q-icon name="settings_ethernet" size="lg" />
            <div class="sensor-name">Wheel Encoders</div>
            <div class="sensor-spec">Differential Drive</div>
            <div class="sensor-freq">50 Hz</div>
          </div>
          <div class="sensor-card map-card">
            <q-icon name="map" size="lg" />
            <div class="sensor-name">Map Server</div>
            <div class="sensor-spec">Static OccupancyGrid</div>
            <div class="sensor-freq">On demand</div>
          </div>
          <div class="sensor-card amcl-card">
            <q-icon name="explore" size="lg" />
            <div class="sensor-name">AMCL Node</div>
            <div class="sensor-spec">2000 particles (KLD)</div>
            <div class="sensor-freq">2 Hz update</div>
          </div>
        </div>

        <!-- METRICS -->
        <div class="suite-metrics">
          <div class="metric-item">
            <div class="metric-label">Localization Error</div>
            <code>~5 cm</code>
          </div>
          <div class="metric-item">
            <div class="metric-label">CPU Usage</div>
            <code>~15%</code>
          </div>
          <div class="metric-item">
            <div class="metric-label">Convergence Time</div>
            <code>~3 sec</code>
          </div>
        </div>
      </div>

      <!-- AMCL YAML CONFIG -->
      <q-expansion-item
        class="bg-slate-800 text-white rounded-lg border border-slate-700 q-mb-xl"
        icon="description"
        label="Ver Configuración Completa: amcl_config.yaml"
        header-class="text-amber-400 font-bold"
      >
        <div class="q-pa-md">
          <CodeBlock
            lang="yaml"
            content="amcl:
  ros__parameters:
    # Filter Parameters
    min_particles: 500
    max_particles: 2000
    kld_err: 0.05
    kld_z: 0.99
    update_min_d: 0.2
    update_min_a: 0.5
    resample_interval: 1
    transform_tolerance: 1.0

    # Laser Model
    laser_model_type: 'likelihood_field'
    laser_max_beams: 60
    laser_z_hit: 0.5
    laser_z_rand: 0.5
    laser_sigma_hit: 0.2
    laser_likelihood_max_dist: 2.0

    # Odometry Model (diff)
    odom_model_type: 'diff'
    odom_alpha1: 0.2
    odom_alpha2: 0.2
    odom_alpha3: 0.2
    odom_alpha4: 0.2

    # Initial Pose
    set_initial_pose: true
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0"
            :copyable="true"
          />
        </div>
      </q-expansion-item>

      <!-- ========================================================================================
         SECTION 8: DOCTORAL CHALLENGE
         Interactive quiz with technical depth
         ======================================================================================== -->
      <q-separator spaced="lg" color="indigo-9" />

      <SectionTitle class="q-mt-xl">10. Doctor's Challenge: The Localization Paradox</SectionTitle>

      <div class="challenge-card">
        <div class="challenge-header">
          <q-icon name="psychology" class="c-icon" />
          <div class="c-info">
            <div class="c-title">🧠 La Paradoja del Pasillo Simétrico</div>
            <div class="c-desc">
              Tu robot está en un pasillo largo con puertas idénticas cada 5 metros. El Lidar ve
              paredes paralelas y puertas, pero todas lucen iguales. AMCL tiene 2000 partículas
              distribuidas uniformemente. Después de 30 segundos moviéndose, las partículas NO
              convergen. ¿Cuál es la causa raíz?
            </div>
          </div>
        </div>

        <div class="challenge-options">
          <div class="c-option correct">
            <div class="opt-radio">A</div>
            <div class="opt-text">
              El entorno es ambiguo (symmetric). El Lidar no puede distinguir entre posiciones
              equivalentes, por lo que múltiples hipótesis tienen la misma likelihood.
              <div class="opt-feedback">
                ¡Correcto! Este es el "Global Localization Problem" en entornos simétricos.
                Solución: agregar landmarks únicos (códigos QR, beacons) o usar múltiples sensores
                (cámara + Lidar).
              </div>
            </div>
          </div>
          <div class="c-option">
            <div class="opt-radio">B</div>
            <div class="opt-text">
              Los parámetros alpha1-4 están mal calibrados, causando que las partículas se dispersen
              demasiado.
            </div>
          </div>
          <div class="c-option">
            <div class="opt-radio">C</div>
            <div class="opt-text">
              El número de partículas (2000) es insuficiente para cubrir todo el pasillo.
            </div>
          </div>
        </div>
      </div>

      <!-- ========================================================================================
         SECTION 9: VIDEO TUTORIAL
         YouTube embedded demo
         ======================================================================================== -->
      <q-separator spaced="lg" color="indigo-9" />

      <SectionTitle class="q-mt-x1">11. Video Tutorial: AMCL Integration</SectionTitle>
      <div class="video-container">
        <div class="video-wrapper">
          <iframe
            width="100%"
            height="100%"
            src="https://youtu.be/Romc22GgusU"
            title="AMCL Tutorial"
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
          Configuración completa de AMCL: Launch files, parámetros, TF tree, sincronización con
          <code>message_filters</code>, y debugging en RViz.
        </div>
      </div>
      <!-- ========================================================================================
         BLOCK I: SUMMARY & BEST PRACTICES
         Final recommendations and checklist
         ======================================================================================== -->
      <q-separator spaced="lg" color="indigo-9" />

      <SectionTitle class="q-mt-xl">📝12. Resumen y Best Practices</SectionTitle>

      <div class="summary-section">
        <div class="summary-grid">
          <div class="summary-card">
            <div class="summary-icon">🎯</div>
            <div class="summary-title">Conceptos Clave</div>
            <ul>
              <li>Bayes Filter: Predicción + Corrección</li>
              <li>Monte Carlo: Aproximación con partículas</li>
              <li>KLD Sampling: Población adaptativa</li>
              <li>Likelihood Field: Modelo de sensor rápido</li>
            </ul>
          </div>

          <div class="summary-card">
            <div class="summary-icon">⚙️</div>
            <div class="summary-title">Parámetros Críticos</div>
            <ul>
              <li><code>min/max_particles</code>: 500-5000</li>
              <li><code>odom_alpha1-4</code>: Ruido de odometría</li>
              <li><code>laser_z_hit/rand</code>: Confianza del sensor</li>
              <li><code>update_min_d/a</code>: Thresholds de update</li>
            </ul>
          </div>

          <div class="summary-card">
            <div class="summary-icon">🚀</div>
            <div class="summary-title">Optimizaciones</div>
            <ul>
              <li>Reduce <code>laser_max_beams</code> a 30-60</li>
              <li>Aumenta <code>resample_interval</code> a 2</li>
              <li>Ajusta <code>update_min_d/a</code> según velocidad</li>
              <li>Usa <code>likelihood_field</code> en lugar de <code>beam</code></li>
            </ul>
          </div>

          <div class="summary-card">
            <div class="summary-icon">⚠️</div>
            <div class="summary-title">Problemas Comunes</div>
            <ul>
              <li>Jitter: Reduce <code>odom_alpha</code> values</li>
              <li>No converge: Aumenta <code>max_particles</code></li>
              <li>CPU alto: Reduce <code>laser_max_beams</code></li>
              <li>Se pierde: Habilita auto-recovery</li>
            </ul>
          </div>
        </div>
      </div>
      <!-- ========================================================================================
         SECTION 10: ENGINEERING SUMMARY
         Key concepts and best practices
         ======================================================================================== -->
      <div class="summary-grid">
        <div class="summary-item">
          <code>Bayes Filter</code>
          <span>Predicción (motion) + Corrección (sensor). Base matemática de AMCL.</span>
        </div>
        <div class="summary-item">
          <code>KLD Sampling</code>
          <span>Ajuste dinámico de partículas basado en incertidumbre. Ahorra CPU.</span>
        </div>
        <div class="summary-item">
          <code>map → odom</code>
          <span>Transform que publica AMCL para corregir drift de odometría.</span>
        </div>
        <div class="summary-item">
          <code>Likelihood Field</code>
          <span>Modelo de sensor rápido. Usa distance transform pre-computado.</span>
        </div>
        <div class="summary-item">
          <code>Global Localization</code>
          <span>Dispersar partículas por todo el mapa. Solución al kidnapped robot.</span>
        </div>
        <div class="summary-item">
          <code>Alpha Parameters</code>
          <span>Modelan ruido de odometría. Calibrar según tipo de suelo.</span>
        </div>
      </div>

      <AlertBlock type="success" title="Best Practices: AMCL Deployment" class="q-mt-lg">
        ✅ <strong>TF Tree:</strong> Verificar con <code>ros2 run tf2_tools view_frames</code> que
        map→odom→base_link esté correcto.
        <br />
        ✅ <strong>Parámetros:</strong> Empezar con defaults, luego ajustar alphas basándote en
        experimentos de drift.
        <br />
        ✅ <strong>Mapa:</strong> Usar mapas limpios sin artefactos. Resolución 0.05m es óptima.
        <br />
        ✅ <strong>Recovery:</strong> Implementar auto-recovery con
        <code>recovery_alpha_slow/fast</code> para detección de secuestro.
        <br />
        ✅ <strong>Performance:</strong> Reducir <code>laser_max_beams</code> a 30-60 si CPU es
        limitada.
      </AlertBlock>
    </div>
  </LessonContainer>
</template>

<script setup lang="ts">
import { ref, computed, onMounted, onUnmounted } from 'vue';
import LessonContainer from 'components/content/LessonContainer.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import MathFormula from 'components/content/MathFormula.vue';

// Particle Filter Simulator State
const numParticles = ref(2000);
const kldError = ref(0.05);
const alpha1 = ref(0.2);
const alpha3 = ref(0.2);
const sensorNoise = ref(0.05);
const zHit = ref(0.5);
const sensorBeamAngle = ref(0);

// Computed metrics
const convergence = computed(() => {
  // Convergence inversely proportional to noise and proportional to particles
  const noiseEffect = (alpha1.value + alpha3.value + sensorNoise.value) / 3;
  const particleEffect = Math.min(numParticles.value / 5000, 1);
  const sensorEffect = zHit.value;
  return Math.max(0, Math.min(100, (1 - noiseEffect) * particleEffect * sensorEffect * 100));
});

const uncertainty = computed(() => {
  // Uncertainty in cm, proportional to noise
  const baseUncertainty = 5; // 5cm baseline
  const noiseMultiplier = 1 + (alpha1.value + alpha3.value + sensorNoise.value) * 10;
  const particleDivisor = Math.max(1, numParticles.value / 1000);
  return (baseUncertainty * noiseMultiplier) / particleDivisor;
});

const cpuLoad = computed(() => {
  // CPU load estimation based on particles and sensor beams
  const particleCost = (numParticles.value / 5000) * 60; // Max 60% for particles
  const sensorCost = (1 - sensorNoise.value) * 20; // Max 20% for sensor processing
  return Math.min(100, particleCost + sensorCost + 10); // +10% baseline
});

const convergenceColor = computed(() => {
  if (convergence.value > 80) return '#10b981'; // green
  if (convergence.value > 50) return '#f59e0b'; // orange
  return '#ef4444'; // red
});

const cpuColor = computed(() => {
  if (cpuLoad.value < 30) return '#10b981'; // green
  if (cpuLoad.value < 60) return '#f59e0b'; // orange
  return '#ef4444'; // red
});

const currentInsight = computed(() => {
  if (numParticles.value < 500) {
    return '⚠️ Muy pocas partículas: El filtro puede perder tracking en entornos complejos.';
  }
  if (numParticles.value > 4000 && cpuLoad.value > 70) {
    return '🔥 Alto uso de CPU: Considera reducir partículas o laser_max_beams.';
  }
  if (alpha1.value > 0.5 || alpha3.value > 0.5) {
    return '📊 Alto ruido de odometría: Las partículas se dispersan mucho. Calibra tus encoders.';
  }
  if (sensorNoise.value > 0.3) {
    return '📡 Alto ruido de sensor: Reduce z_hit o mejora la calidad del mapa.';
  }
  if (convergence.value > 90) {
    return '✅ Excelente configuración: Alta convergencia con CPU razonable.';
  }
  if (zHit.value < 0.3) {
    return '⚠️ Baja confianza en sensor: Aumenta z_hit si tu Lidar es confiable.';
  }
  return '💡 Ajusta los parámetros y observa cómo cambia la distribución de partículas.';
});

// Preset configurations
const applyPreset = (preset: string) => {
  switch (preset) {
    case 'indoor':
      numParticles.value = 1500;
      kldError.value = 0.05;
      alpha1.value = 0.15;
      alpha3.value = 0.15;
      sensorNoise.value = 0.05;
      zHit.value = 0.7;
      break;
    case 'outdoor':
      numParticles.value = 3000;
      kldError.value = 0.03;
      alpha1.value = 0.4;
      alpha3.value = 0.4;
      sensorNoise.value = 0.15;
      zHit.value = 0.5;
      break;
    case 'warehouse':
      numParticles.value = 2500;
      kldError.value = 0.04;
      alpha1.value = 0.25;
      alpha3.value = 0.3;
      sensorNoise.value = 0.08;
      zHit.value = 0.6;
      break;
    case 'default':
      numParticles.value = 2000;
      kldError.value = 0.05;
      alpha1.value = 0.2;
      alpha3.value = 0.2;
      sensorNoise.value = 0.05;
      zHit.value = 0.5;
      break;
  }
};

// Animate sensor beam
let beamInterval: NodeJS.Timeout | null = null;
onMounted(() => {
  beamInterval = setInterval(() => {
    sensorBeamAngle.value += 0.05;
    if (sensorBeamAngle.value > Math.PI * 2) {
      sensorBeamAngle.value = 0;
    }
  }, 50);
});

onUnmounted(() => {
  if (beamInterval) {
    clearInterval(beamInterval);
  }
});
</script>

<style scoped>
/* ============================================================================
   HERO SECTION STYLES
   ============================================================================ */
.hero-section {
  background:
    radial-gradient(circle at 20% 80%, rgba(139, 92, 246, 0.15), transparent 50%),
    linear-gradient(135deg, #0f172a 0%, #1e293b 100%);
  border-radius: 24px;
  padding: 4rem 3rem;
  border: 1px solid rgba(148, 163, 184, 0.1);
  box-shadow: 0 20px 50px -10px rgba(0, 0, 0, 0.5);
  position: relative;
  overflow: hidden;
  display: grid;
  grid-template-columns: 1.5fr 1fr;
  gap: 3rem;
  align-items: center;
  max-width: 1400px;
  margin-left: auto;
  margin-right: auto;
}

.hero-section::before {
  content: '';
  position: absolute;
  top: -50%;
  right: -20%;
  width: 1000px;
  height: 600px;
  background: radial-gradient(circle, rgba(59, 130, 246, 0.15) 0%, transparent 70%);
  border-radius: 50%;
  animation: pulse-glow 4s ease-in-out infinite;
}

@keyframes pulse-glow {
  0%,
  100% {
    transform: scale(1);
    opacity: 0.3;
  }
  50% {
    transform: scale(1.1);
    opacity: 0.5;
  }
}

.hero-content {
  position: relative;
  z-index: 2;
}

.hero-badge {
  display: inline-flex;
  align-items: center;
  gap: 0.5rem;
  background: rgba(139, 92, 246, 0.15);
  border: 1px solid rgba(139, 92, 246, 0.3);
  color: #a78bfa;
  padding: 0.5rem 1rem;
  border-radius: 99px;
  font-size: 0.85rem;
  font-weight: 700;
  letter-spacing: 1px;
  margin-bottom: 1.5rem;
}

.hero-title {
  font-size: 3.5rem;
  font-weight: 900;
  line-height: 1.1;
  margin: 0 0 1.5rem 0;
  letter-spacing: -1px;
}

.gradient-text {
  background: linear-gradient(to right, #a78bfa, #22d3ee);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
}

.hero-subtitle {
  font-size: 1.1rem;
  line-height: 1.7;
  color: #cbd5e1;
  max-width: 600px;
  margin-bottom: 2rem;
}

.hero-stats {
  display: flex;
  gap: 2rem;
}

.stat-item {
  display: flex;
  flex-direction: column;
  gap: 0.25rem;
}

.stat-val {
  font-size: 2rem;
  font-weight: 900;
  color: #3b82f6;
  line-height: 1;
}

.stat-label {
  font-size: 0.75rem;
  color: #94a3b8;
  text-transform: uppercase;
  letter-spacing: 1px;
}

.hero-viz {
  position: relative;
  z-index: 2;
}

.particle-cloud {
  width: 100%;
  height: auto;
  filter: drop-shadow(0 0 20px rgba(59, 130, 246, 0.3));
}

.robot-pos {
  animation: robot-pulse 2s ease-in-out infinite;
}

@keyframes robot-pulse {
  0%,
  100% {
    r: 8;
  }
  50% {
    r: 10;
  }
}

.robot-ring {
  animation: ring-expand 2s ease-in-out infinite;
}

@keyframes ring-expand {
  0%,
  100% {
    r: 15;
    opacity: 0.3;
  }
  50% {
    r: 20;
    opacity: 0.1;
  }
}

.particle {
  animation: particle-float 3s ease-in-out infinite;
}

@keyframes particle-float {
  0%,
  100% {
    opacity: 0.6;
    transform: scale(1);
  }
  50% {
    opacity: 1;
    transform: scale(1.2);
  }
}

.prob-wave {
  animation: wave-expand 4s ease-in-out infinite;
}

.w1 {
  animation-delay: 0s;
}
.w2 {
  animation-delay: 0.5s;
}
.w3 {
  animation-delay: 1s;
}

@keyframes wave-expand {
  0% {
    r: 30;
    opacity: 0;
  }
  50% {
    opacity: 0.2;
  }
  100% {
    r: 120;
    opacity: 0;
  }
}

/* ============================================================================
   CONTENT BLOCKS
   ============================================================================ */
.content-block {
  margin-bottom: 4rem;
  max-width: 1400px;
  margin-left: auto;
  margin-right: auto;
}

/* ============================================================================
   THEORY SECTION
   ============================================================================ */
.theory-section {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 16px;
  padding: 2rem;
  max-width: 1400px;
  margin-left: auto;
  margin-right: auto;
}

.theory-header {
  display: flex;
  align-items: center;
  gap: 1rem;
  font-size: 1.5rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 2rem;
}

.equations-grid {
  display: grid;
  grid-template-columns: repeat(2, minmax(300px, 1fr));
  gap: 2rem;
}

.equation-card {
  background: rgba(30, 41, 59, 0.8);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.equation-card.prediction {
  border-top: 4px solid #3b82f6;
}

.equation-card.correction {
  border-top: 4px solid #10b981;
}

.eq-title {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  font-weight: 700;
  font-size: 1.1rem;
  color: #f1f5f9;
}

.eq-formula {
  background: rgba(0, 0, 0, 0.4);
  padding: 1.5rem;
  border-radius: 8px;
  font-family: 'Times New Roman', serif;
  font-size: 1.1rem;
  text-align: center;
  color: #a5f3fc;
  overflow-x: auto;
}

.eq-explanation {
  font-size: 0.9rem;
  line-height: 1.7;
  color: #cbd5e1;
}

.eq-params {
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
  padding-top: 1rem;
  border-top: 1px solid rgba(148, 163, 184, 0.1);
}

.param-item {
  display: flex;
  flex-direction: column;
  gap: 0.25rem;
}

.param-item code {
  color: #6ee7b7;
  font-family: 'Fira Code', monospace;
  font-size: 0.9rem;
}

.param-item span {
  color: #94a3b8;
  font-size: 0.8rem;
}

/* Monte Carlo Section */
.mc-section {
  background: linear-gradient(135deg, rgba(139, 92, 246, 0.05) 0%, rgba(59, 130, 246, 0.05) 100%);
  border: 1px solid rgba(139, 92, 246, 0.2);
  border-radius: 16px;
  padding: 2rem;
  max-width: 1400px;
  margin-left: auto;
  margin-right: auto;
}

.section-subtitle {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  font-size: 1.3rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 1.5rem;
}

.mc-concept-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  gap: 1.5rem;
}

.concept-card {
  background: rgba(30, 41, 59, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 12px;
  padding: 1.5rem;
  text-align: center;
  transition: all 0.3s ease;
}

.concept-card:hover {
  transform: translateY(-4px);
  border-color: rgba(139, 92, 246, 0.4);
  box-shadow: 0 10px 30px rgba(139, 92, 246, 0.2);
}

.concept-card {
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
}

.concept-icon {
  font-size: 3rem;
  margin-bottom: 1rem;
}

.concept-title {
  font-weight: 700;
  font-size: 1.1rem;
  color: #f1f5f9;
  margin-bottom: 0.75rem;
}

.concept-desc {
  font-size: 0.85rem;
  line-height: 1.6;
  color: #cbd5e1;
}

/* Algorithm Box */
.algorithm-box {
  background: rgba(0, 0, 0, 0.4);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
}

.algo-header {
  background: rgba(59, 130, 246, 0.1);
  border-bottom: 1px solid rgba(59, 130, 246, 0.3);
  padding: 1rem 1.5rem;
  display: flex;
  align-items: center;
  gap: 0.75rem;
  font-weight: 700;
  color: #60a5fa;
}

.algo-content {
  padding: 1.5rem;
}

.pseudocode {
  font-family: 'Fira Code', monospace;
  font-size: 0.9rem;
  line-height: 1.8;
  color: #e2e8f0;
  margin: 0;
}

.pseudocode .keyword {
  color: #c084fc;
  font-weight: 700;
}

.pseudocode .comment {
  color: #6b7280;
  font-style: italic;
}

/* ============================================================================
   NEW SECTIONS STYLES
   ============================================================================ */

/* Section Divider */
.section-divider {
  margin: 4rem 0;
}

/* Comparison Grid (Sensor Models) */
.comparison-grid {
  display: grid;
  grid-template-columns: repeat(2, minmax(300px, 1fr));
  gap: 2rem;
}

.comparison-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
}

.comparison-card:hover {
  transform: translateY(-4px);
  box-shadow: 0 10px 30px rgba(59, 130, 246, 0.15);
}

.comparison-card.beam {
  border-top: 4px solid #f59e0b;
}

.comparison-card.likelihood {
  border-top: 4px solid #10b981;
}

.comparison-card .card-header {
  display: flex;
  align-items: center;
  gap: 1rem;
  font-size: 1.3rem;
  font-weight: 700;
  color: #f1f5f9;
}

.card-pros,
.card-cons {
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
}

.pros-title,
.cons-title {
  font-weight: 700;
  font-size: 1rem;
  color: #cbd5e1;
}

.card-pros ul,
.card-cons ul {
  margin: 0;
  padding-left: 1.5rem;
  color: #94a3b8;
  font-size: 0.9rem;
  line-height: 1.7;
}

/* Parameter Categories */
.param-categories {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
  gap: 1.5rem;
}

.category-card {
  background: rgba(30, 41, 59, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 12px;
  padding: 1.5rem;
  text-align: center;
  transition: all 0.3s ease;
}

.category-card:hover {
  transform: translateY(-4px) scale(1.02);
  box-shadow: 0 15px 40px rgba(59, 130, 246, 0.25);
}

.category-card {
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
}

.category-card.filter {
  border-top: 4px solid #3b82f6;
}

.category-card.laser {
  border-top: 4px solid #10b981;
}

.category-card.odom {
  border-top: 4px solid #a855f7;
}

.category-card.init {
  border-top: 4px solid #f59e0b;
}

.cat-icon {
  font-size: 2.5rem;
  margin-bottom: 1rem;
}

.cat-title {
  font-weight: 700;
  font-size: 1.1rem;
  color: #f1f5f9;
  margin-bottom: 0.5rem;
}

.cat-desc {
  font-size: 0.85rem;
  color: #94a3b8;
  margin-bottom: 1rem;
  line-height: 1.5;
}

.cat-count {
  font-size: 0.8rem;
  color: #64748b;
  font-weight: 600;
}

/* TF Tree Container */
.tf-tree-container {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 16px;
  overflow: hidden;
  max-width: 1400px;
  margin-left: auto;
  margin-right: auto;
}

.tree-header {
  background: rgba(59, 130, 246, 0.1);
  border-bottom: 1px solid rgba(59, 130, 246, 0.3);
  padding: 1rem 1.5rem;
  display: flex;
  align-items: center;
  gap: 0.75rem;
  font-weight: 700;
  font-size: 1.2rem;
  color: #60a5fa;
}

.tree-content {
  padding: 2rem;
}

/* Recovery Grid */
.recovery-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  gap: 1.5rem;
}

.recovery-method {
  background: rgba(30, 41, 59, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 12px;
  padding: 1.5rem;
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
}

.recovery-method:hover {
  transform: translateY(-4px);
  border-color: rgba(59, 130, 246, 0.4);
  box-shadow: 0 10px 30px rgba(59, 130, 246, 0.2);
}

.method-header {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  font-weight: 700;
  font-size: 1.1rem;
  color: #f1f5f9;
  margin-bottom: 1rem;
}

.method-body {
  font-size: 0.9rem;
  line-height: 1.6;
  color: #cbd5e1;
}

/* Tool Container */
.tool-container {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 16px;
  overflow: hidden;
  max-width: 1400px;
  margin-left: auto;
  margin-right: auto;
}

.tool-header {
  background: rgba(139, 92, 246, 0.1);
  border-bottom: 1px solid rgba(139, 92, 246, 0.3);
  padding: 1rem 1.5rem;
  display: flex;
  align-items: center;
  gap: 0.75rem;
  font-weight: 700;
  font-size: 1.2rem;
  color: #a78bfa;
}

.tool-body {
  padding: 2rem;
}

/* Optimization Grid */
.optimization-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
  gap: 1.5rem;
}

.opt-card {
  background: rgba(30, 41, 59, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-left: 4px solid #10b981;
  border-radius: 12px;
  padding: 1.5rem;
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
}

.opt-card:hover {
  transform: translateX(4px);
  border-left-color: #22c55e;
  box-shadow: -4px 0 20px rgba(16, 185, 129, 0.2);
}

.opt-icon {
  font-size: 2rem;
  margin-bottom: 0.75rem;
}

.opt-title {
  font-weight: 700;
  font-size: 1.1rem;
  color: #f1f5f9;
  margin-bottom: 0.75rem;
}

.opt-desc {
  font-size: 0.9rem;
  line-height: 1.6;
  color: #cbd5e1;
}

/* Summary Section */
.summary-section {
  background: linear-gradient(135deg, rgba(59, 130, 246, 0.05) 0%, rgba(16, 185, 129, 0.05) 100%);
  border: 1px solid rgba(59, 130, 246, 0.2);
  border-radius: 16px;
  padding: 2rem;
}

.summary-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  gap: 1.5rem;
}

.summary-card {
  background: rgba(30, 41, 59, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 12px;
  padding: 1.5rem;
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
}

.summary-card:hover {
  transform: translateY(-2px);
  border-color: rgba(59, 130, 246, 0.3);
  box-shadow: 0 8px 20px rgba(59, 130, 246, 0.15);
}

.summary-icon {
  font-size: 2.5rem;
  margin-bottom: 1rem;
}

.summary-title {
  font-weight: 700;
  font-size: 1.2rem;
  color: #f1f5f9;
  margin-bottom: 1rem;
}

.summary-card ul {
  margin: 0;
  padding-left: 1.5rem;
  color: #cbd5e1;
  font-size: 0.9rem;
  line-height: 1.8;
}

.summary-card code {
  color: #6ee7b7;
  font-family: 'Fira Code', monospace;
  font-size: 0.85rem;
  background: rgba(0, 0, 0, 0.3);
  padding: 0.2rem 0.4rem;
  border-radius: 4px;
}

/* ============================================================================
   CASE STUDY SECTION
   ============================================================================ */
.sensor-suite-viz {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 16px;
  padding: 2rem;
  max-width: 1400px;
  margin-left: auto;
  margin-right: auto;
}

.suite-header {
  display: flex;
  align-items: center;
  gap: 1rem;
  margin-bottom: 2rem;
}

.suite-title {
  font-size: 1.5rem;
  font-weight: 700;
  color: #f1f5f9;
}

.suite-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
  gap: 1.5rem;
  margin-bottom: 2rem;
}

.sensor-card {
  background: rgba(30, 41, 59, 0.8);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  text-align: center;
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
}

.sensor-card:hover {
  transform: translateY(-4px);
  box-shadow: 0 10px 30px rgba(59, 130, 246, 0.25);
}

.sensor-card.lidar-card {
  border-top: 4px solid #22c55e;
}

.sensor-card.encoder-card {
  border-top: 4px solid #3b82f6;
}

.sensor-card.map-card {
  border-top: 4px solid #a855f7;
}

.sensor-card.amcl-card {
  border-top: 4px solid #f59e0b;
}

.sensor-name {
  font-weight: 700;
  font-size: 1.1rem;
  color: #f1f5f9;
  margin: 1rem 0 0.5rem 0;
}

.sensor-spec {
  font-size: 0.85rem;
  color: #94a3b8;
  margin-bottom: 0.5rem;
}

.sensor-freq {
  font-size: 0.8rem;
  color: #64748b;
  font-weight: 600;
}

.suite-metrics {
  display: flex;
  justify-content: space-around;
  gap: 2rem;
  padding-top: 2rem;
  border-top: 1px solid rgba(148, 163, 184, 0.1);
}

.metric-item {
  text-align: center;
}

.metric-label {
  font-size: 0.85rem;
  color: #94a3b8;
  margin-bottom: 0.5rem;
}

.metric-item code {
  font-size: 1.2rem;
  color: #22d3ee;
  font-weight: 700;
}

/* ============================================================================
   DOCTORAL CHALLENGE
   ============================================================================ */
.challenge-card {
  background: linear-gradient(135deg, rgba(139, 92, 246, 0.05) 0%, rgba(59, 130, 246, 0.05) 100%);
  border: 1px solid rgba(139, 92, 246, 0.3);
  border-radius: 16px;
  padding: 2rem;
  max-width: 1400px;
  margin-left: auto;
  margin-right: auto;
}

.challenge-header {
  display: flex;
  gap: 1.5rem;
  margin-bottom: 2rem;
}

.c-icon {
  font-size: 3rem;
  color: #a78bfa;
}

.c-info {
  flex: 1;
}

.c-title {
  font-size: 1.5rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 1rem;
}

.c-desc {
  font-size: 1rem;
  line-height: 1.7;
  color: #cbd5e1;
}

.challenge-options {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.c-option {
  background: rgba(30, 41, 59, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  display: flex;
  gap: 1rem;
  transition: all 0.3s ease;
  cursor: pointer;
}

.c-option:hover {
  border-color: rgba(139, 92, 246, 0.4);
  background: rgba(30, 41, 59, 0.8);
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

/* ============================================================================
   VIDEO SECTION (FINAL POSITION)
   ============================================================================ */
.video-container {
  background: #0f172a;
  border: 1px solid #334155;
  border-radius: 12px;
  overflow: hidden;
  padding: 1rem;
  box-shadow: 0 10px 30px -5px rgba(0, 0, 0, 0.5);
  max-width: 1400px;
  margin-left: auto;
  margin-right: auto;
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

/* ============================================================================
   SUMMARY ITEMS
   ============================================================================ */
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

.section-group {
  margin-bottom: 3rem;
  max-width: 1400px;
  margin-left: auto;
  margin-right: auto;
}

/* Responsive */
.summary-section {
  background: linear-gradient(135deg, rgba(59, 130, 246, 0.05) 0%, rgba(16, 185, 129, 0.05) 100%);
  border: 1px solid rgba(59, 130, 246, 0.2);
  border-radius: 16px;
  padding: 2rem;
}

.summary-grid {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 1.5rem;
}

.summary-card {
  background: rgba(30, 41, 59, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 12px;
  padding: 1.5rem;
}

.summary-icon {
  font-size: 2.5rem;
  margin-bottom: 1rem;
}

.summary-title {
  font-weight: 700;
  font-size: 1.2rem;
  color: #f1f5f9;
  margin-bottom: 1rem;
}

.summary-card ul {
  margin: 0;
  padding-left: 1.5rem;
  color: #cbd5e1;
  font-size: 0.9rem;
  line-height: 1.8;
}

.summary-card code {
  color: #6ee7b7;
  font-family: 'Fira Code', monospace;
  font-size: 0.85rem;
  background: rgba(0, 0, 0, 0.3);
  padding: 0.2rem 0.4rem;
  border-radius: 4px;
}

/* Responsive */
@media (max-width: 1024px) {
  .hero-section {
    grid-template-columns: 1fr;
    padding: 3rem 1.5rem;
  }

  .hero-title {
    font-size: 2.5rem;
  }

  .video-container {
    grid-template-columns: 1fr;
  }

  .equations-grid,
  .mc-concept-grid,
  .comparison-grid,
  .param-categories,
  .recovery-grid,
  .optimization-grid,
  .summary-grid {
    grid-template-columns: 1fr;
  }
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2rem;
  }

  .hero-subtitle {
    font-size: 1rem;
  }

  .video-section {
    padding: 1.5rem;
  }

  .video-title {
    font-size: 1.4rem;
  }

  .desc-list li {
    font-size: 0.9rem;
  }
}

@media (max-width: 600px) {
  .hero-stats {
    flex-direction: column;
    gap: 1rem;
  }

  .hero-title {
    font-size: 1.75rem;
  }

  .stat-val {
    font-size: 1.5rem;
  }

  .video-section {
    padding: 1rem;
  }

  .video-header {
    flex-direction: column;
    align-items: flex-start;
    gap: 0.5rem;
  }

  .video-title {
    font-size: 1.2rem;
  }

  .video-description {
    padding: 1rem;
  }

  .desc-title {
    font-size: 1rem;
  }

  .theory-section,
  .mc-section {
    padding: 1.5rem;
  }

  .section-subtitle {
    font-size: 1.1rem;
  }
}

/* ============================================================================
   PARAMETER TABLE STYLES
   ============================================================================ */
.param-table-container {
  background: #0f172a;
  border-radius: 16px;
  overflow: hidden;
  border: 1px solid #334155;
  max-width: 1400px;
  margin-left: auto;
  margin-right: auto;
}

.table-title {
  background: #1e293b;
  padding: 1.5rem;
  font-weight: 700;
  font-size: 1.3rem;
  color: #fff;
  border-bottom: 2px solid #3b82f6;
  display: flex;
  align-items: center;
  gap: 0.75rem;
}

.table-wrapper {
  max-height: 600px;
  overflow-y: auto;
  overflow-x: auto;
}

.param-table {
  width: 100%;
  border-collapse: collapse;
  min-width: 1000px;
}

.param-table thead {
  background: rgba(30, 41, 59, 0.9);
  position: sticky;
  top: 0;
  z-index: 10;
}

.param-table th {
  padding: 1rem;
  text-align: left;
  color: #94a3b8;
  font-weight: 600;
  font-size: 0.85rem;
  text-transform: uppercase;
  letter-spacing: 0.5px;
  border-bottom: 2px solid #3b82f6;
  white-space: nowrap;
}

.param-table td {
  padding: 1rem;
  color: #cbd5e1;
  border-top: 1px solid rgba(51, 65, 85, 0.5);
  font-size: 0.9rem;
  vertical-align: top;
}

.param-table code {
  background: rgba(59, 130, 246, 0.15);
  padding: 0.3rem 0.6rem;
  border-radius: 6px;
  color: #60a5fa;
  font-family: 'Fira Code', monospace;
  font-weight: 600;
  font-size: 0.85rem;
  white-space: nowrap;
}

.param-table tbody tr {
  transition: background 0.2s ease;
}

.param-table tbody tr:not(.category-row):hover {
  background: rgba(59, 130, 246, 0.05);
}

.category-row {
  background: linear-gradient(90deg, rgba(139, 92, 246, 0.15) 0%, rgba(59, 130, 246, 0.15) 100%);
  font-weight: 700;
  color: #a78bfa;
  position: sticky;
  top: 57px;
  z-index: 5;
}

.category-row td {
  padding: 0.75rem 1rem;
  border-top: 2px solid #8b5cf6;
  border-bottom: 2px solid #8b5cf6;
  font-size: 1rem;
  letter-spacing: 0.5px;
}

.table-footer {
  background: rgba(30, 41, 59, 0.6);
  border-top: 1px solid #334155;
  padding: 1.5rem;
}

.footer-note {
  color: #cbd5e1;
  font-size: 0.9rem;
  line-height: 1.7;
}

.footer-note strong {
  color: #fbbf24;
}

.footer-note code {
  background: rgba(59, 130, 246, 0.1);
  padding: 0.2rem 0.5rem;
  border-radius: 4px;
  color: #60a5fa;
  font-family: 'Fira Code', monospace;
  font-size: 0.85rem;
}

/* Scrollbar styling for table */
.table-wrapper::-webkit-scrollbar {
  width: 10px;
  height: 10px;
}

.table-wrapper::-webkit-scrollbar-track {
  background: #1e293b;
  border-radius: 5px;
}

.table-wrapper::-webkit-scrollbar-thumb {
  background: #3b82f6;
  border-radius: 5px;
}

.table-wrapper::-webkit-scrollbar-thumb:hover {
  background: #60a5fa;
}

/* ============================================================================
   TF TREE DIAGRAM STYLES
   ============================================================================ */
.tf-tree-viz {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 16px;
  padding: 2rem;
}

.tree-canvas {
  background: rgba(0, 0, 0, 0.3);
  border-radius: 12px;
  padding: 1.5rem;
  margin-bottom: 2rem;
}

.tf-diagram {
  width: 100%;
  height: auto;
}

.tf-frame {
  cursor: pointer;
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
}

.tf-frame:hover rect {
  filter: brightness(1.3);
  stroke-width: 4;
}

.tf-frame:hover text {
  filter: brightness(1.2);
}

.tree-legend {
  background: rgba(30, 41, 59, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 12px;
  padding: 1.5rem;
}

.legend-title {
  font-size: 1.1rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 1rem;
  text-align: center;
}

.legend-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 1rem;
}

.legend-item {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  padding: 0.75rem;
  background: rgba(0, 0, 0, 0.2);
  border-radius: 8px;
  transition: all 0.3s ease;
}

.legend-item:hover {
  background: rgba(0, 0, 0, 0.4);
  transform: translateX(4px);
}

.legend-color {
  width: 24px;
  height: 24px;
  border-radius: 6px;
  border: 2px solid rgba(255, 255, 255, 0.3);
  flex-shrink: 0;
}

.legend-info {
  display: flex;
  flex-direction: column;
  gap: 0.25rem;
}

.legend-name {
  font-family: 'Fira Code', monospace;
  font-weight: 700;
  font-size: 0.9rem;
  color: #f1f5f9;
}

.legend-desc {
  font-size: 0.8rem;
  color: #94a3b8;
  line-height: 1.4;
}

/* ============================================================================
   PARTICLE FILTER SIMULATOR STYLES
   ============================================================================ */
.tool-wrapper {
  background: linear-gradient(135deg, #0f172a 0%, #1e293b 100%);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  overflow: hidden;
}

.tool-header {
  background: rgba(59, 130, 246, 0.1);
  border-bottom: 1px solid rgba(59, 130, 246, 0.3);
  padding: 1.5rem;
  display: flex;
  align-items: center;
  gap: 1rem;
}

.tool-header .q-icon {
  font-size: 2rem;
  color: #60a5fa;
}

.tool-title {
  font-size: 1.3rem;
  font-weight: 700;
  color: #f1f5f9;
}

.tool-subtitle {
  font-size: 0.9rem;
  color: #94a3b8;
  margin-top: 0.25rem;
}

.particle-lab {
  display: grid;
  grid-template-columns: 350px 1fr;
  min-height: 600px;
}

.lab-controls {
  background: rgba(30, 41, 59, 0.6);
  border-right: 1px solid rgba(148, 163, 184, 0.1);
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 2rem;
  overflow-y: auto;
  max-height: 800px;
}

.control-section {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.section-label {
  font-size: 0.95rem;
  font-weight: 700;
  color: #a78bfa;
  text-transform: uppercase;
  letter-spacing: 1px;
  margin-bottom: 0.5rem;
  padding-bottom: 0.5rem;
  border-bottom: 2px solid rgba(167, 139, 250, 0.2);
}

.control-group {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.c-label {
  display: flex;
  align-items: center;
  justify-content: space-between;
  font-size: 0.85rem;
  color: #cbd5e1;
  font-weight: 600;
}

.c-label .q-icon {
  cursor: help;
}

.c-value {
  font-family: 'Fira Code', monospace;
  font-size: 0.9rem;
  color: #22d3ee;
  font-weight: 700;
  text-align: right;
}

.presets-section {
  margin-top: auto;
  padding-top: 1rem;
  border-top: 1px solid rgba(148, 163, 184, 0.1);
}

.preset-buttons {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 0.5rem;
  margin-top: 0.75rem;
}

.preset-btn {
  font-size: 0.8rem;
  font-weight: 600;
  padding: 0.5rem;
  border-radius: 8px;
  transition: all 0.3s ease;
}

.preset-btn:hover {
  transform: translateY(-2px);
  box-shadow: 0 4px 12px rgba(59, 130, 246, 0.3);
}

.lab-viz {
  padding: 2rem;
  background: linear-gradient(180deg, #1e293b 0%, #0f172a 100%);
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.viz-header {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  font-size: 1.1rem;
  font-weight: 700;
  color: #f1f5f9;
  padding-bottom: 1rem;
  border-bottom: 2px solid rgba(59, 130, 246, 0.3);
}

.viz-header .q-icon {
  font-size: 1.5rem;
  color: #60a5fa;
}

.particle-plot {
  flex: 1;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.particle-canvas {
  width: 100%;
  height: auto;
  background: rgba(0, 0, 0, 0.4);
  border-radius: 12px;
  border: 1px solid rgba(148, 163, 184, 0.1);
}

.sensor-beam {
  animation: beam-sweep 3s linear infinite;
}

@keyframes beam-sweep {
  0%,
  100% {
    opacity: 0.2;
  }
  50% {
    opacity: 0.6;
  }
}

.plot-stats {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 1rem;
  padding: 1rem;
  background: rgba(30, 41, 59, 0.6);
  border-radius: 12px;
  border: 1px solid rgba(148, 163, 184, 0.1);
}

.stat-item {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.5rem;
}

.stat-label {
  font-size: 0.75rem;
  color: #94a3b8;
  text-transform: uppercase;
  letter-spacing: 0.5px;
  font-weight: 600;
}

.stat-value {
  font-size: 1.5rem;
  font-weight: 700;
  font-family: 'Fira Code', monospace;
}

.insights-panel {
  background: linear-gradient(135deg, rgba(139, 92, 246, 0.1) 0%, rgba(59, 130, 246, 0.1) 100%);
  border: 1px solid rgba(139, 92, 246, 0.3);
  border-radius: 12px;
  padding: 1.5rem;
}

.insight-title {
  font-size: 1rem;
  font-weight: 700;
  color: #a78bfa;
  margin-bottom: 0.75rem;
}

.insight-text {
  font-size: 0.9rem;
  color: #cbd5e1;
  line-height: 1.6;
}

/* Responsive adjustments */
@media (max-width: 1024px) {
  .particle-lab {
    grid-template-columns: 1fr;
  }

  .lab-controls {
    border-right: none;
    border-bottom: 1px solid rgba(148, 163, 184, 0.1);
    max-height: none;
  }

  .preset-buttons {
    grid-template-columns: repeat(4, 1fr);
  }
}

/* ============================================================================
   PSEUDOCODE COMPONENT STYLES
   ============================================================================ */
.pseudocode-container {
  background: #0f172a;
  border-radius: 12px;
  padding: 1.5rem;
  font-family: 'Fira Code', monospace;
  font-size: 0.9rem;
  color: #e2e8f0;
  border: 1px solid rgba(148, 163, 184, 0.1);
  overflow-x: auto;
}

.code-line {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  line-height: 1.6;
  white-space: nowrap;
}

.indent-1 {
  padding-left: 1.5rem;
}
.indent-2 {
  padding-left: 3rem;
}

.spacer {
  height: 0.5rem;
}

.keyword {
  color: #c678dd;
  font-weight: 700;
} /* Purple/Pink */
.func-name {
  color: #61afef;
  font-weight: 700;
} /* Blue */
.comment {
  color: #5c6370;
  font-style: italic;
} /* Grey */

/* Integration with MathFormula inside code */
.pseudocode-container .math-formula {
  font-size: 0.95em; /* Adjust to match code font size */
  color: #e5c07b; /* Gold/Yellow for variables */
}
</style>
