<template>
  <LessonContainer>
    <!-- HERO INTRO -->
    <TextBlock>
      <strong>AMCL (Adaptive Monte Carlo Localization)</strong> es la implementación estándar en ROS
      2 para localización probabilística en 2D. Utiliza un <strong>Filtro de Partículas</strong>
      para estimar la posición y orientación $(x, y, \theta)$ del robot en un mapa conocido.
    </TextBlock>

    <AlertBlock type="info" title="Filosofía Bayesiana">
      El robot nunca sabe "dónde está". Solo tiene una <strong>Creencia</strong> (Belief)
      representada por una nube de puntos.
      <br />
      <em
        >"Estoy 90% seguro que estoy en la cocina, pero hay un 10% de probabilidad de que esté en el
        pasillo".</em
      >
    </AlertBlock>

    <!-- AMCL MATH -->
    <div class="section-group">
      <SectionTitle>1. Teoría Matemática (Bayes Filter)</SectionTitle>
      <div class="math-grid q-mt-md">
        <div class="math-card prediction">
          <div class="math-header">
            <q-icon name="directions_run" />
            <span>1. Predicción (Motion Model)</span>
          </div>
          <div class="formula">$Bel^-(x_t) = \int p(x_t | u_t, x_{t-1}) Bel(x_{t-1}) dx_{t-1}$</div>
          <div class="explanation">
            Movemos todas las partículas según la odometría ($u_t$) + ruido. La incertidumbre crece
            (la nube se dispersa).
            <br />
            Configurado con <code>alpha1..alpha4</code>.
          </div>
        </div>

        <div class="math-card correction">
          <div class="math-header">
            <q-icon name="visibility" />
            <span>2. Corrección (Sensor Model)</span>
          </div>
          <div class="formula">$Bel(x_t) = \eta p(z_t | x_t) Bel^-(x_t)$</div>
          <div class="explanation">
            Asignamos un peso ($w$) a cada partícula comparando el escaneo láser real ($z_t$) con el
            mapa virtual.
            <br />
            Configurado con <code>z_hit</code>, <code>z_rand</code>...
          </div>
        </div>

        <div class="math-card resampling">
          <div class="math-header">
            <q-icon name="casino" />
            <span>3. Resampling (La Ruleta)</span>
          </div>
          <div class="explanation">
            Supervivencia del más apto. Las partículas con peso bajo mueren. Las de peso alto se
            duplican.
            <br />
            Resultado: La nube se concentra (converge) en la posición real.
          </div>
        </div>
      </div>
    </div>

    <!-- PARAMETER TUNING -->
    <div class="section-group">
      <SectionTitle>2. Advanced Parameter Tuning</SectionTitle>
      <TextBlock>
        El tuning incorrecto causa "saltos" (jitter) o que el robot se pierda fácilmente.
      </TextBlock>

      <div class="tuning-grid q-mt-md">
        <!-- PARTICLES -->
        <div class="tuning-card blue">
          <div class="card-title">Particle Count (KLD Sampling)</div>
          <div class="card-desc">
            AMCL ajusta dinámicamente el número de partículas basado en la incertidumbre (algoritmo
            KLD).
          </div>
          <div class="params-list">
            <div class="param">
              <code>min_particles</code> (Default: 500)
              <span>Mínimo para mantener estabilidad. <br />Recom: 500-2000.</span>
            </div>
            <div class="param">
              <code>max_particles</code> (Default: 2000)
              <span>Tope para no saturar CPU. <br />Recom: 3000-5000 (PC potente).</span>
            </div>
          </div>
        </div>

        <!-- ODOMETRY -->
        <div class="tuning-card purple">
          <div class="card-title">Odometry Noise (Alphas)</div>
          <div class="card-desc">
            Define cuánto "desconfiamos" de las ruedas. Si es muy bajo, AMCL no corregirá el drift.
            Si es muy alto, las partículas se dispersan demasiado.
          </div>
          <div class="params-list">
            <div class="param">
              <code>alpha1</code> (Rot -> Rot)
              <span>Error de rotación generado por rotación.</span>
            </div>
            <div class="param">
              <code>alpha2</code> (Trans -> Rot)
              <span>Error de rotación generado por traslación.</span>
            </div>
            <div class="param">
              <code>alpha3</code> (Trans -> Trans)
              <span>Error de traslación generado por traslación.</span>
            </div>
          </div>
        </div>

        <!-- LASER -->
        <div class="tuning-card green">
          <div class="card-title">Laser Model (Likelihood)</div>
          <div class="card-desc">Modelo de probabilidad del sensor.</div>
          <div class="params-list">
            <div class="param">
              <code>z_hit</code> (Default: 0.5)
              <span>Confianza en mediciones correctas.</span>
            </div>
            <div class="param">
              <code>z_rand</code> (Default: 0.5)
              <span>Ruido aleatorio (fantasmas, gente).</span>
            </div>
            <div class="param">
              <code>sigma_hit</code> (Std Dev)
              <span>Incertidumbre del láser (ruido gaussiano).</span>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- KIDNAPPED ROBOT -->
    <div class="section-group">
      <SectionTitle>3. The Kidnapped Robot Problem</SectionTitle>
      <AlertBlock type="warning" title="Recuperación">
        Si mueves el robot manualmente ("Secuestro"), las partículas divergen y AMCL falla.
        <br />
        <strong>Solución:</strong> Relocalización Global.
      </AlertBlock>

      <div class="recovery-methods q-mt-md">
        <div class="method-box">
          <div class="method-title">Opción A: GUI (RViz)</div>
          <div class="method-content">
            Herramienta <strong>2D Pose Estimate</strong>. Haces clic manual en el mapa.
          </div>
        </div>

        <div class="method-box code">
          <div class="method-title">Opción B: Service Call (Programático)</div>
          <div class="method-code">
            <CodeBlock
              lang="bash"
              content="ros2 service call /reinitialize_global_localization nav2_msgs/srv/LoadMap '{}'"
              :copyable="true"
            />
            <div class="note">
              Esto dispersa partículas aleatoriamente por TODO el mapa (Global Localization). El
              robot debe moverse para converger.
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- TF TREE -->
    <div class="section-group">
      <SectionTitle>4. El Árbol de Transformadas (TF)</SectionTitle>
      <TextBlock>
        La única responsabilidad de AMCL es publicar la transformación <strong>map -> odom</strong>.
        Esto corrige el drift acumulado por la odometría pura.
      </TextBlock>

      <div class="tf-viz q-mt-md">
        <div class="tf-node map">/map</div>
        <div class="tf-link amcl">
          <span class="link-label">AMCL Update</span>
          <span class="link-arrow">⬇</span>
        </div>
        <div class="tf-node odom">/odom</div>
        <div class="tf-link wheel">
          <span class="link-label">Wheel Encoders</span>
          <span class="link-arrow">⬇</span>
        </div>
        <div class="tf-node base">/base_link</div>
        <div class="tf-link static">
          <span class="link-label">URDF</span>
          <span class="link-arrow">⬇</span>
        </div>
        <div class="tf-node laser">/laser_link</div>
      </div>
    </div>

    <!-- ANIMATION -->
    <div class="section-group">
      <SectionTitle>5. Visualización de Convergencia</SectionTitle>
      <div class="convergence-viz-container">
        <div class="viz-stage">
          <div class="robot-target"></div>

          <!-- CSS Particles -->
          <div class="particle p1"></div>
          <div class="particle p2"></div>
          <div class="particle p3"></div>
          <div class="particle p4"></div>
          <div class="particle p5"></div>
          <div class="particle p6"></div>

          <div class="sensor-fan"></div>
        </div>
        <div class="viz-info">
          <div class="info-badge">Estado: CONVERGIENDO</div>
          <div class="info-text">
            Al detectar características únicas (esquinas), las partículas incorrectas mueren.
          </div>
        </div>
      </div>
    </div>

    <!-- DIDACTIC MINIGAME -->
    <div class="section-group">
      <SectionTitle>6. Minijuego: Detective de Partículas</SectionTitle>

      <div class="challenge-card">
        <div class="challenge-title">🕵️ Caso: El Pasillo Maldito</div>
        <div class="challenge-desc">
          El robot está en un pasillo largo, recto y sin puertas. El láser ve paredes a ambos lados.
          ¿Qué le pasa a la nube de partículas?
        </div>

        <div class="options-grid">
          <div class="option wrong">
            <div class="opt-head">A. Converge rápido</div>
            <div class="opt-body">Imposible, no hay referencias únicas en X.</div>
          </div>
          <div class="option correct">
            <div class="opt-head">B. Se estira infinito</div>
            <div class="opt-body">
              Exacto. En el eje Y (ancho) sabe dónde está, pero en X (largo) la incertidumbre es
              total. La nube se vuelve una línea larga.
            </div>
          </div>
          <div class="option wrong">
            <div class="opt-head">C. Se dispersa circularmente</div>
            <div class="opt-body">No, porque las paredes laterales restringen la rotación y Y.</div>
          </div>
        </div>
      </div>
    </div>

    <!-- RESUMEN -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen Técnico</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>MCL</code>
          <span>Monte Carlo Localization (Bayes Filter)</span>
        </div>
        <div class="summary-item">
          <code>KLD Sampling</code>
          <span>Ajuste dinámico de población de partículas</span>
        </div>
        <div class="summary-item">
          <code>map -> odom</code>
          <span>Transformada que publica AMCL</span>
        </div>
        <div class="summary-item">
          <code>Likelihood Field</code>
          <span>Modelo de sensor para LIDAR</span>
        </div>
        <div class="summary-item">
          <code>Global Localization</code>
          <span>Reiniciar partículas en todo el mapa</span>
        </div>
      </div>

      <AlertBlock type="success" title="Best Practices" class="q-mt-lg">
        ✅ Si el mapa es muy grande, aumenta <code>max_particles</code>.
        <br />
        ✅ Ajusta <code>update_min_d</code> (0.2m) y <code>update_min_a</code> (0.5rad) para ahorrar
        CPU cuando el robot está quieto.
        <br />
        ✅ Usa el modelo <code>likelihood_field</code> para Lidars, es más suave que
        <code>beam_model</code>.
      </AlertBlock>
    </div>
  </LessonContainer>
</template>

<script setup lang="ts">
import LessonContainer from 'components/content/LessonContainer.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
</script>

<style scoped>
.section-group {
  margin-bottom: 3.5rem;
}

/* MATH GRID */
.math-grid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 1.5rem;
}

.math-card {
  background: rgba(15, 23, 42, 0.8);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.math-header {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  font-weight: 700;
  color: #f1f5f9;
  font-size: 1rem;
}

.formula {
  background: rgba(0, 0, 0, 0.3);
  padding: 0.75rem;
  border-radius: 8px;
  font-family: 'Times New Roman', serif;
  font-style: italic;
  font-size: 0.9rem;
  text-align: center;
  color: #a5f3fc;
  overflow-x: auto;
}

.explanation {
  font-size: 0.85rem;
  color: #94a3b8;
  line-height: 1.5;
}

/* TUNING GRID */
.tuning-grid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 1.5rem;
}

.tuning-card {
  background: rgba(30, 41, 59, 0.5);
  border: 1px solid rgba(255, 255, 255, 0.05);
  border-radius: 12px;
  padding: 1.5rem;
}

.tuning-card.blue {
  border-top: 4px solid #3b82f6;
}
.tuning-card.purple {
  border-top: 4px solid #a855f7;
}
.tuning-card.green {
  border-top: 4px solid #10b981;
}

.card-title {
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 0.5rem;
  font-size: 1.1rem;
}

.card-desc {
  font-size: 0.85rem;
  color: #cbd5e1;
  margin-bottom: 1.5rem;
}

.params-list {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.param {
  display: flex;
  flex-direction: column;
  gap: 0.25rem;
  font-family: 'Fira Code', monospace;
  color: #e2e8f0;
  font-size: 0.8rem;
}

.param span {
  font-family: sans-serif;
  color: #94a3b8;
  font-size: 0.75rem;
}

/* RECOVERY */
.recovery-methods {
  display: grid;
  grid-template-columns: 1fr 2fr;
  gap: 1.5rem;
}

.method-box {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 8px;
  padding: 1rem;
}

.method-title {
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 0.5rem;
}

.method-content {
  font-size: 0.9rem;
  color: #cbd5e1;
}

.note {
  font-size: 0.8rem;
  color: #94a3b8;
  margin-top: 0.5rem;
  font-style: italic;
}

/* TF VIZ */
.tf-viz {
  display: flex;
  flex-direction: column;
  align-items: center;
  background: rgba(15, 23, 42, 0.8);
  padding: 2rem;
  border-radius: 12px;
}

.tf-node {
  background: #334155;
  color: #f1f5f9;
  padding: 0.5rem 1rem;
  border-radius: 6px;
  border: 1px solid #475569;
  font-family: 'Fira Code', monospace;
}

.tf-node.map {
  background: #1e293b;
  border-color: #3b82f6;
}
.tf-node.odom {
  background: #1e293b;
  border-color: #a855f7;
}
.tf-node.base {
  background: #1e293b;
  border-color: #f59e0b;
}

.tf-link {
  display: flex;
  flex-direction: column;
  align-items: center;
  margin: 0.5rem 0;
}

.link-label {
  font-size: 0.7rem;
  color: #94a3b8;
  margin-bottom: -5px;
}

.link-arrow {
  color: #64748b;
  font-size: 1.5rem;
}

/* ANIMATION */
.convergence-viz-container {
  background: #000;
  border-radius: 12px;
  height: 300px;
  position: relative;
  overflow: hidden;
  border: 1px solid rgba(16, 185, 129, 0.3);
}

.viz-stage {
  position: relative;
  width: 100%;
  height: 100%;
}

.robot-target {
  width: 20px;
  height: 20px;
  background: #10b981;
  border-radius: 50%;
  position: absolute;
  top: 50%;
  left: 50%;
  transform: translate(-50%, -50%);
  box-shadow: 0 0 20px rgba(16, 185, 129, 0.5);
  z-index: 10;
}

.particle {
  width: 5px;
  height: 5px;
  background: #ef4444;
  border-radius: 50%;
  position: absolute;
  top: 50%;
  left: 50%;
  opacity: 0.6;
  animation: convergeParticles 4s infinite ease-in-out;
}

.p1 {
  animation-delay: 0s;
  --angle: 0deg;
  --dist: 100px;
}
.p2 {
  animation-delay: 0.2s;
  --angle: 60deg;
  --dist: 120px;
}
.p3 {
  animation-delay: 0.4s;
  --angle: 120deg;
  --dist: 90px;
}
.p4 {
  animation-delay: 0.6s;
  --angle: 180deg;
  --dist: 110px;
}
.p5 {
  animation-delay: 0.8s;
  --angle: 240deg;
  --dist: 130px;
}
.p6 {
  animation-delay: 1s;
  --angle: 300deg;
  --dist: 80px;
}

@keyframes convergeParticles {
  0% {
    transform: translate(-50%, -50%) rotate(var(--angle)) translateY(var(--dist)) scale(1);
    opacity: 0;
  }
  20% {
    opacity: 0.8;
  }
  80% {
    transform: translate(-50%, -50%) rotate(var(--angle)) translateY(10px) scale(0.5);
    opacity: 1;
  }
  100% {
    transform: translate(-50%, -50%) rotate(var(--angle)) translateY(10px) scale(0);
    opacity: 0;
  }
}

.viz-info {
  position: absolute;
  bottom: 20px;
  left: 50%;
  transform: translateX(-50%);
  text-align: center;
}

.info-badge {
  background: rgba(16, 185, 129, 0.2);
  color: #10b981;
  padding: 4px 12px;
  border-radius: 99px;
  font-size: 0.8rem;
  font-weight: 700;
  display: inline-block;
  margin-bottom: 0.5rem;
}

.info-text {
  font-size: 0.8rem;
  color: #94a3b8;
}

/* SUMMARY */
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
  color: #6ee7b7;
  font-family: 'Fira Code', monospace;
  font-size: 0.95rem;
}

.summary-item span {
  color: #cbd5e1;
  font-size: 0.85rem;
}

/* RESPONSIVE */
@media (max-width: 1024px) {
  .math-grid,
  .tuning-grid,
  .recovery-methods {
    grid-template-columns: 1fr;
  }

  .tf-viz {
    padding: 1rem;
  }
}

@media (max-width: 600px) {
  .convergence-viz-container {
    height: 200px;
  }
}
</style>
