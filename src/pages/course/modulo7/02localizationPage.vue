<template>
  <q-page class="q-pa-lg column items-center">
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-teal-4 text-weight-bold q-mb-sm">
          MÓDULO 7.2: LOCALIZACIÓN PROBABILÍSTICA
        </div>
        <h1 class="hero-title">
          AMCL: En busca de la <span class="text-white">Ubicación Perdida</span>
        </h1>
        <TextBlock>
          Tener un mapa no sirve de nada si no sabes dónde estás en él. El GPS no funciona en
          interiores y la odometría acumula errores.
          <br /><br />
          <strong>AMCL (Adaptive Monte Carlo Localization)</strong> es un algoritmo probabilístico
          que usa un "filtro de partículas" para adivinar la posición del robot comparando lo que
          <em>ve</em> (Lidar) con lo que <em>debería ver</em> según el mapa.
        </TextBlock>
      </div>
    </section>

    <div class="section-group self-stretch">
      <SectionTitle>1. La Nube de Posibilidades</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            AMCL no rastrea <em>un</em> robot. Rastrea
            <strong>miles de robots virtuales</strong> (partículas). <br /><br />
            Cada punto rojo es una hipótesis:
            <em>"Creo que estoy en (x=2, y=5) mirando al Norte"</em>. <br /><br />
            Al principio, el robot tiene <strong>Incertidumbre Global</strong>: las partículas están
            por todas partes.
          </TextBlock>
        </template>
        <template #right>
          <div
            class="tool-card bg-slate-900 relative-position overflow-hidden full-height border-teal"
          >
            <div class="absolute-center text-center z-top pointer-events-none">
              <div class="text-h6 text-white text-shadow">Incertidumbre</div>
              <div class="text-caption text-grey-4 text-shadow">"¿Dónde estoy?"</div>
            </div>

            <div class="particle-container">
              <div v-for="n in 40" :key="n" class="particle" :style="getRandomStyle()"></div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>2. El Ciclo de Vida (Supervivencia)</SectionTitle>
      <TextBlock>
        El algoritmo es un bucle infinito de selección natural. Solo sobreviven los que coinciden
        con la realidad.
      </TextBlock>

      <div class="row q-col-gutter-lg q-mt-md">
        <div class="col-12 col-md-4">
          <div class="custom-card border-blue q-pa-md text-center full-height transition-hover">
            <q-icon name="directions_run" color="blue-4" size="3rem" class="q-mb-sm" />
            <div class="text-h6 text-blue-1">1. Movimiento</div>
            <p class="text-caption text-grey-4 q-mt-sm">
              Cuando el robot real se mueve, movemos todas las partículas igual (según odometría) +
              un poco de ruido aleatorio.
            </p>
          </div>
        </div>

        <div class="col-12 col-md-4">
          <div class="custom-card border-purple q-pa-md text-center full-height transition-hover">
            <q-icon name="scale" color="purple-4" size="3rem" class="q-mb-sm" />
            <div class="text-h6 text-purple-1">2. Pesaje (Score)</div>
            <p class="text-caption text-grey-4 q-mt-sm">
              Comparamos Lidar Real vs Mapa Virtual.
              <br />
              <span class="text-green-4">Coincide = Peso Alto</span><br />
              <span class="text-red-4">No coincide = Peso Bajo</span>
            </p>
          </div>
        </div>

        <div class="col-12 col-md-4">
          <div class="custom-card border-green q-pa-md text-center full-height transition-hover">
            <q-icon name="casino" color="green-4" size="3rem" class="q-mb-sm" />
            <div class="text-h6 text-green-1">3. Ruleta (Resample)</div>
            <p class="text-caption text-grey-4 q-mt-sm">
              Matamos las partículas malas y clonamos las buenas. La nube se concentra donde es más
              probable estar.
            </p>
          </div>
        </div>
      </div>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>3. Convergencia: De la Duda a la Certeza</SectionTitle>
      <TextBlock>
        Observa cómo la nube (flechas rojas) se "encoge" alrededor de la posición real (verde) a
        medida que el robot reconoce el entorno.
      </TextBlock>

      <div
        class="tool-card bg-black q-pa-lg q-mt-md relative-position overflow-hidden"
        style="height: 300px"
      >
        <div class="absolute-center">
          <div class="robot-real bg-green-5 shadow-green"></div>

          <div class="particle-group">
            <div class="p-arrow" style="--angle: 0deg; --delay: 0s"></div>
            <div class="p-arrow" style="--angle: 45deg; --delay: 0.1s"></div>
            <div class="p-arrow" style="--angle: 90deg; --delay: 0.2s"></div>
            <div class="p-arrow" style="--angle: 135deg; --delay: 0.3s"></div>
            <div class="p-arrow" style="--angle: 180deg; --delay: 0.4s"></div>
            <div class="p-arrow" style="--angle: 225deg; --delay: 0.5s"></div>
            <div class="p-arrow" style="--angle: 270deg; --delay: 0.6s"></div>
            <div class="p-arrow" style="--angle: 315deg; --delay: 0.7s"></div>
          </div>
        </div>

        <div class="absolute-bottom text-center q-mb-md">
          <div class="status-badge font-mono text-xs">Estado: CONVERGIENDO...</div>
        </div>
      </div>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>4. El "Dedazo" de Dios (2D Pose Estimate)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            A veces el robot está totalmente perdido ("Secuestrado"). Necesita ayuda divina.
            <br /><br />
            En RViz, usa el botón <strong>2D Pose Estimate</strong>. Haces clic en el mapa donde
            <em>sabes</em> que está el robot y arrastras la flecha para indicar hacia dónde mira.
            <br /><br />
            Esto resetea todas las partículas alrededor de tu clic.
          </TextBlock>
        </template>
        <template #right>
          <div
            class="tool-card bg-slate-800 flex flex-center border-teal relative-position overflow-hidden"
          >
            <div class="column items-center full-width">
              <div
                class="rviz-btn bg-slate-700 q-px-md q-py-sm rounded-borders text-white q-mb-xl cursor-pointer hover-scale border-light shadow-2"
              >
                <q-icon name="navigation" class="rotate-90 q-mr-sm" color="green-4" />
                2D Pose Estimate
              </div>

              <div class="text-caption text-grey-5 font-italic q-mb-lg">"¡Estás aquí!"</div>

              <div class="mouse-cursor absolute">
                <q-icon name="near_me" color="white" size="lg" class="drop-shadow cursor-icon" />
                <div class="click-ripple"></div>
              </div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>5. Tuning: Ajustando la Magia</SectionTitle>
      <TextBlock>
        AMCL se configura en el archivo <code>.yaml</code>. Estos son los parámetros que debes
        conocer:
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <div class="col-12 col-md-6">
          <q-list bordered separator class="bg-slate-900 rounded-borders border-teal-dim">
            <q-item>
              <q-item-section>
                <q-item-label class="text-teal-4 font-mono font-bold">min_particles</q-item-label>
                <q-item-label caption class="text-grey-4">
                  Mínimo de hipótesis (ej: 500). Menos = más rápido, pero menos robusto.
                </q-item-label>
              </q-item-section>
            </q-item>
            <q-item>
              <q-item-section>
                <q-item-label class="text-teal-4 font-mono font-bold">max_particles</q-item-label>
                <q-item-label caption class="text-grey-4">
                  Máximo (ej: 2000). Cuidado: subir esto consume mucha CPU.
                </q-item-label>
              </q-item-section>
            </q-item>
          </q-list>
        </div>
        <div class="col-12 col-md-6">
          <q-list bordered separator class="bg-slate-900 rounded-borders border-teal-dim">
            <q-item>
              <q-item-section>
                <q-item-label class="text-teal-4 font-mono font-bold">update_min_d</q-item-label>
                <q-item-label caption class="text-grey-4">
                  Moverse X metros antes de recalcular (ej: 0.2m). No gastes CPU si el robot no se
                  mueve.
                </q-item-label>
              </q-item-section>
            </q-item>
            <q-item>
              <q-item-section>
                <q-item-label class="text-teal-4 font-mono font-bold">update_min_a</q-item-label>
                <q-item-label caption class="text-grey-4">
                  Girar X radianes antes de recalcular (ej: 0.5 rad). Girar aporta mucha información
                  al Lidar.
                </q-item-label>
              </q-item-section>
            </q-item>
          </q-list>
        </div>
      </div>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>6. El Fenómeno del Teletransporte</SectionTitle>
      <AlertBlock type="warning" title="Saltos en el Mapa">
        A veces verás que el robot "salta" de golpe un metro en RViz. <br />
        Esto es normal. Significa que AMCL encontró una posición mejor y corrigió el error acumulado
        de la odometría (/odom -> /map).
      </AlertBlock>
    </div>

    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>7. Troubleshooting</SectionTitle>

      <div class="row q-col-gutter-md">
        <div class="col-12">
          <q-expansion-item
            icon="rotate_right"
            label="El robot está perdido, ¿qué hago?"
            header-class="bg-slate-800 text-white rounded-borders border-light"
            class="q-mb-sm shadow-1"
          >
            <div class="q-pa-md bg-slate-900 text-grey-4 border-light border-top-0">
              <strong>Solución:</strong> Gira el robot en su propio eje 360 grados. El Lidar verá
              toda la geometría de la habitación y las partículas convergerán rápidamente.
            </div>
          </q-expansion-item>

          <q-expansion-item
            icon="blur_circular"
            label="Las partículas oscilan mucho (Jitter)"
            header-class="bg-slate-800 text-white rounded-borders border-light"
            class="q-mb-sm shadow-1"
          >
            <div class="q-pa-md bg-slate-900 text-grey-4 border-light border-top-0">
              Posibles causas: 1. El mapa no coincide con la realidad (muebles movidos). 2.
              Parámetros de ruido de odometría (alpha1...alpha4) demasiado altos.
            </div>
          </q-expansion-item>
        </div>
      </div>
    </div>
  </q-page>
</template>

<script setup lang="ts">
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import SplitBlock from 'components/content/SplitBlock.vue';

// Helper visualización
const getRandomStyle = () => {
  const top = Math.random() * 90 + 5; // Evitar bordes extremos
  const left = Math.random() * 90 + 5;
  const delay = Math.random() * 2;
  return {
    top: `${top}%`,
    left: `${left}%`,
    animationDelay: `${delay}s`,
  };
};
</script>

<style scoped>
/* --- ESTILOS MAESTROS (Corregidos e Inyectados) --- */

.intro-hero,
.section-group {
  width: 100%;
  max-width: 1100px;
  margin: 0 auto 3.5rem auto;
}

.intro-hero {
  padding: 3rem 2rem;
  background:
    radial-gradient(circle at center, rgba(45, 212, 191, 0.2), transparent 60%),
    /* Teal Gradient */ rgba(15, 23, 42, 0.8);
  backdrop-filter: blur(20px);
  border-radius: 24px;
  border: 1px solid rgba(148, 163, 184, 0.2);
  text-align: center;
}

.hero-title {
  font-size: 3rem;
  font-weight: 800;
  margin: 0 0 1.5rem 0;
  line-height: 1.1;
  color: #f8fafc;
}

/* CARDS & CONTAINERS */
.tool-card {
  height: 100%;
  border-radius: 16px;
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(255, 255, 255, 0.05);
}

.custom-card {
  border-radius: 16px;
  background: rgba(30, 41, 59, 0.5);
  border-top: 4px solid;
  height: 100%;
  border-left: 1px solid rgba(255, 255, 255, 0.05);
  border-right: 1px solid rgba(255, 255, 255, 0.05);
  border-bottom: 1px solid rgba(255, 255, 255, 0.05);
}

.full-height {
  height: 100%;
}
.full-width {
  width: 100%;
}
.pointer-events-none {
  pointer-events: none;
}
.z-top {
  z-index: 10;
}
.text-shadow {
  text-shadow: 0 2px 4px rgba(0, 0, 0, 0.8);
}

/* BORDERS & COLORS */
.border-teal {
  border-left: 4px solid #2dd4bf;
}
.border-teal-dim {
  border: 1px solid rgba(45, 212, 191, 0.3);
}
.border-blue {
  border-top-color: #3b82f6;
}
.border-purple {
  border-top-color: #a855f7;
}
.border-green {
  border-top-color: #4ade80;
}
.border-light {
  border: 1px solid rgba(255, 255, 255, 0.1);
}
.border-top-0 {
  border-top: 0;
}

.shadow-green {
  box-shadow: 0 0 15px #4ade80;
}

/* ANIMATION 1: PARTICLES (Floating) */
.particle-container {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
}
.particle {
  position: absolute;
  width: 6px;
  height: 6px;
  background: #f87171; /* Red hypothesis */
  border-radius: 50%;
  opacity: 0.6;
  box-shadow: 0 0 4px #f87171;
  animation: floatRandom 4s infinite ease-in-out;
}
@keyframes floatRandom {
  0%,
  100% {
    transform: translate(0, 0);
  }
  50% {
    transform: translate(15px, -15px);
  }
}

/* ANIMATION 2: CONVERGENCE */
.robot-real {
  width: 20px;
  height: 20px;
  border-radius: 50%;
  border: 2px solid #fff;
  position: absolute;
  z-index: 5;
  /* Centering */
  top: 50%;
  left: 50%;
  transform: translate(-50%, -50%);
}
.p-arrow {
  position: absolute;
  top: 50%;
  left: 50%; /* Center origin */
  width: 0;
  height: 0;
  border-left: 6px solid transparent;
  border-right: 6px solid transparent;
  border-bottom: 18px solid #f87171;
  /* El truco: Rotamos, trasladamos lejos (dispersión) y luego acercamos (convergencia) */
  animation: converge 3s infinite ease-in-out;
}
@keyframes converge {
  0% {
    transform: translate(-50%, -50%) rotate(var(--angle)) translateY(120px) scale(1);
    opacity: 0.3;
  }
  50% {
    transform: translate(-50%, -50%) rotate(var(--angle)) translateY(25px) scale(0.8);
    opacity: 1;
  }
  80% {
    transform: translate(-50%, -50%) rotate(var(--angle)) translateY(25px) scale(0.8);
    opacity: 1;
  }
  100% {
    transform: translate(-50%, -50%) rotate(var(--angle)) translateY(10px) scale(0);
    opacity: 0;
  }
}

.status-badge {
  background: rgba(45, 212, 191, 0.1);
  color: #2dd4bf;
  padding: 4px 12px;
  border-radius: 99px;
  border: 1px solid rgba(45, 212, 191, 0.3);
}

/* ANIMATION 3: CLICK ESTIMATE */
.mouse-cursor {
  top: 60%;
  left: 60%;
  animation: mouseMove 3s infinite;
}
.cursor-icon {
  filter: drop-shadow(2px 4px 4px rgba(0, 0, 0, 0.5));
}

.click-ripple {
  position: absolute;
  top: -10px;
  left: -10px;
  width: 40px;
  height: 40px;
  border-radius: 50%;
  border: 2px solid #4ade80;
  opacity: 0;
  animation: rippleEffect 3s infinite;
}

@keyframes mouseMove {
  0%,
  100% {
    transform: translate(0, 0);
  }
  20% {
    transform: translate(-40px, -40px);
  } /* Move to target */
  30% {
    transform: translate(-40px, -40px) scale(0.9);
  } /* Click Down */
  40% {
    transform: translate(-40px, -40px) scale(1);
  } /* Click Up */
  60% {
    transform: translate(0, 0);
  } /* Return */
}

@keyframes rippleEffect {
  0%,
  25% {
    transform: scale(0);
    opacity: 0;
  }
  30% {
    transform: translate(-40px, -40px) scale(0);
    opacity: 1;
  }
  40% {
    transform: translate(-40px, -40px) scale(1.5);
    opacity: 0;
  }
  100% {
    opacity: 0;
  }
}

/* UTILS */
.bg-slate-900 {
  background: #0f172a;
}
.bg-slate-800 {
  background: #1e293b;
}
.bg-slate-700 {
  background: #334155;
}
.font-mono {
  font-family: 'Fira Code', monospace;
}
.text-xs {
  font-size: 0.8rem;
}
.font-bold {
  font-weight: 700;
}
.transition-hover {
  transition: transform 0.2s;
}
.transition-hover:hover {
  transform: translateY(-5px);
}
.hover-scale:hover {
  transform: scale(1.05);
  background: #475569;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
