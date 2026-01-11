<template>
  <q-page class="q-pa-lg column items-center">
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-indigo-4 text-weight-bold q-mb-sm">
          MÓDULO 7.3: EL CEREBRO DE NAVEGACIÓN
        </div>
        <h1 class="hero-title">Nav2: Configuración y <span class="text-white">Tuning</span></h1>
        <TextBlock>
          Instalar Nav2 es fácil. Hacer que funcione bien es un arte. Por defecto, tu robot
          probablemente se moverá a tirones, girará en círculos o se quedará atascado en puertas.
          <br /><br />
          En esta lección, abriremos el "cerebro" de navegación para ajustar sus dos hemisferios: el
          <strong>Planificador Global</strong> (Estrategia) y el
          <strong>Controlador Local</strong> (Táctica).
        </TextBlock>
      </div>
    </section>

    <div class="section-group self-stretch">
      <SectionTitle>1. El GPS y el Chofer</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Nav2 divide el problema de ir de A a B en dos tareas independientes que corren en
            paralelo:
          </TextBlock>
          <ul class="q-pl-md text-grey-4 tool-list">
            <li>
              🌍 <strong>Global Planner:</strong> Calcula la ruta completa al inicio (o cada pocos
              segundos). Usa el mapa estático. Ignora obstáculos dinámicos repentinos.
            </li>
            <li>
              🏎️ <strong>Local Planner (Controller):</strong> Calcula la velocidad
              (<code>cmd_vel</code>) 20 veces por segundo. Intenta seguir la ruta global esquivando
              lo que ve el Lidar <em>ahora mismo</em>.
            </li>
          </ul>
        </template>
        <template #right>
          <div class="tool-card bg-slate-900 relative-position overflow-hidden h-full">
            <div class="absolute-center full-width text-center">
              <svg width="300" height="200" class="absolute-center">
                <path
                  d="M 20 180 Q 100 20, 280 20"
                  fill="none"
                  stroke="#3b82f6"
                  stroke-width="4"
                  stroke-dasharray="10,5"
                  style="opacity: 0.5"
                />
              </svg>

              <div class="robot-nav bg-indigo-5 shadow-indigo"></div>

              <div class="local-path-curve"></div>

              <div class="obstacle-dynamic bg-red-5"></div>
            </div>

            <div class="legend row absolute-bottom justify-center q-pb-sm">
              <span class="text-blue-4 text-caption q-mr-md">Global Plan</span>
              <span class="text-green-4 text-caption">Local Control</span>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>2. Costmaps: El Mapa de Riesgos</SectionTitle>
      <TextBlock>
        El robot no ve "sillas" o "mesas". Ve <strong>Costos</strong>. Un Costmap es una rejilla
        donde cada celda tiene un valor de peligro (0 a 255). Nav2 usa dos costmaps separados:
      </TextBlock>

      <div class="row q-col-gutter-lg q-mt-md">
        <div class="col-12 col-md-6">
          <div class="custom-card border-blue q-pa-md">
            <div class="text-h6 text-blue-1">Global Costmap</div>
            <p class="text-caption text-grey-4">
              Cubre todo el mapa. Se usa para planificar la ruta larga.
              <br />Se alimenta del mapa estático (<code>/map</code>).
            </p>
          </div>
        </div>
        <div class="col-12 col-md-6">
          <div class="custom-card border-green q-pa-md">
            <div class="text-h6 text-green-1">Local Costmap</div>
            <p class="text-caption text-grey-4">
              Es una ventana pequeña (ej: 3x3 metros) que se mueve con el robot (Rolling Window).
              <br />Se alimenta del Lidar en tiempo real.
            </p>
          </div>
        </div>
      </div>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>3. Las Capas del Pastel (Layers)</SectionTitle>
      <TextBlock>
        Un Costmap se construye apilando capas de información. El orden importa.
      </TextBlock>

      <div class="tool-card bg-slate-800 q-pa-lg flex flex-center column">
        <div class="layer-stack bg-purple-9 text-white shadow-3">
          Inflation Layer (El "Halo" de seguridad)
        </div>
        <div class="layer-arrow">⬇️</div>
        <div class="layer-stack bg-red-8 text-white shadow-2">Obstacle Layer (Lidar en vivo)</div>
        <div class="layer-arrow">⬇️</div>
        <div class="layer-stack bg-grey-7 text-black shadow-1">Static Layer (El Mapa guardado)</div>
      </div>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>4. Inflation Layer: El Espacio Personal</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            El concepto más crítico de Tuning. Si el robot pasa rozando la pared, podría
            engancharse. La <strong>Capa de Inflación</strong> añade un "gradiente de miedo"
            alrededor de cada obstáculo. <br /><br />
            El planificador intentará mantenerse en la zona azul (costo 0) y evitará la roja (costo
            letal).
          </TextBlock>
        </template>
        <template #right>
          <div class="tool-card bg-white relative-position overflow-hidden h-full flex flex-center">
            <div class="obstacle-center bg-black"></div>

            <div class="circle-layer layer-lethal"></div>

            <div class="circle-layer layer-inflation"></div>

            <div class="absolute-bottom text-center text-black text-caption q-pb-xs font-bold">
              Gradiente de Costo
            </div>
          </div>
        </template>
      </SplitBlock>

      <div class="q-mt-md">
        <div class="code-label yaml">nav2_params.yaml</div>
        <CodeBlock
          lang="yaml"
          code="inflation_layer:
  plugin: 'nav2_costmap_2d::InflationLayer'
  inflation_radius: 0.55  # Hasta dónde llega el miedo
  cost_scaling_factor: 3.0 # Qué tan rápido baja el miedo"
        />
      </div>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>5. Configurando el Global Planner</SectionTitle>
      <TextBlock>
        El estándar es <strong>NavFn</strong> (Dijkstra/A*). Busca el camino más corto
        matemáticamente.
      </TextBlock>
      <AlertBlock type="info" title="Smac Planner">
        Para robots más avanzados (coches tipo Ackerman o robots industriales), se usa
        <strong>Smac Planner</strong>, que entiende de cinemática (sabe que un coche no puede girar
        en su propio eje).
      </AlertBlock>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>6. Configurando el Controller (DWB)</SectionTitle>
      <TextBlock>
        Aquí es donde ajustas la "personalidad" de conducción. ¿Agresiva o suave? El controlador
        estándar es <strong>DWB (Dynamic Window Approach)</strong>. Simula múltiples trayectorias
        futuras y elige la mejor.
      </TextBlock>

      <div class="row q-gutter-md q-mt-sm">
        <div class="col-12 col-md-5">
          <CodeBlock
            lang="yaml"
            title="Velocidades"
            code="max_vel_x: 0.26
min_vel_x: 0.0
max_vel_theta: 1.0 # Rad/s
acc_lim_x: 2.5     # Aceleración
acc_lim_theta: 3.2"
          />
        </div>
        <div class="col-12 col-md-6">
          <AlertBlock type="danger" title="¡Cuidado con la Aceleración!">
            Si pones <code>acc_lim</code> muy alto en simulación, el robot se moverá perfecto. En la
            realidad, si el motor no tiene tanta potencia, el controlador se desincronizará y el
            robot empezará a oscilar.
            <strong>Ajusta esto a la física real de tus motores.</strong>
          </AlertBlock>
        </div>
      </div>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>7. Behavior Trees: El Flujo de Decisión</SectionTitle>
      <TextBlock>
        Nav2 no es solo "ir de A a B". Es un árbol de decisiones (XML). Define qué hacer si algo
        falla.
      </TextBlock>

      <div class="tool-card bg-slate-900 q-pa-lg flex flex-center">
        <div class="column items-center">
          <div class="bt-node bg-indigo-6 text-white">Main Sequence</div>
          <div class="bt-line"></div>
          <div class="row q-gutter-lg">
            <div class="column items-center">
              <div class="bt-node bg-green-6 text-white">Compute Path</div>
            </div>
            <div class="column items-center">
              <div class="bt-node bg-green-6 text-white">Follow Path</div>
              <div class="bt-line-red"></div>
              <div class="bt-node bg-red-5 text-white">Recovery?</div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>8. Recoveries: El Plan B</SectionTitle>
      <TextBlock>
        ¿Qué pasa si el robot se queda encerrado o choca? Nav2 ejecuta comportamientos de
        recuperación:
      </TextBlock>
      <div class="row q-gutter-sm q-mt-sm">
        <q-chip icon="refresh" color="orange-9" text-color="white">Spin (Girar 360)</q-chip>
        <q-chip icon="arrow_back" color="orange-9" text-color="white">BackUp (Retroceder)</q-chip>
        <q-chip icon="wifi_tethering" color="orange-9" text-color="white">Clear Costmap</q-chip>
      </div>
      <TextBlock class="q-mt-sm">
        El "Clear Costmap" es útil cuando el robot alucina un obstáculo que ya no está (un fantasma
        en el mapa).
      </TextBlock>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>9. Síntoma: El Robot Borracho</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Si tu robot avanza haciendo "S" (oscilando izquierda/derecha), suele ser culpa de:
          </TextBlock>
          <ul class="q-pl-md text-grey-4 tool-list">
            <li>
              <strong>Lookahead Distance:</strong> El robot mira un punto demasiado cercano en la
              ruta. Auméntalo.
            </li>
            <li><strong>Frecuencia de Control:</strong> Muy baja (ej: 5Hz). Súbela a 20Hz.</li>
            <li>
              <strong>Latencia:</strong> Tu computadora va lenta. Baja el <code>max_vel</code>.
            </li>
          </ul>
        </template>
        <template #right>
          <div class="tool-card bg-slate-900 relative-position overflow-hidden h-full">
            <div class="absolute-center">
              <svg width="200" height="150" class="overflow-visible">
                <path
                  d="M 10 140 Q 50 10, 100 140 T 190 10"
                  fill="none"
                  stroke="#f87171"
                  stroke-width="3"
                  class="path-wobble"
                />
              </svg>
              <div class="robot-wobble bg-indigo-5"></div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>10. Obsesión por la Precisión</SectionTitle>
      <AlertBlock type="warning" title="xy_goal_tolerance & yaw_goal_tolerance">
        Si pides una precisión de 1cm (0.01m) y tu robot es grande y torpe, nunca llegará. Se
        quedará bailando alrededor de la meta intentando ajustarse eternamente.
        <br /><strong>Recomendación:</strong> 0.15m (15cm) y 0.1 rad es suficiente para la mayoría.
      </AlertBlock>
    </div>
  </q-page>
</template>

<script setup lang="ts">
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
</script>

<style scoped>
/* GENERAL */
.intro-hero,
.section-group {
  width: 100%;
  max-width: 1100px;
  margin: 0 auto 3.5rem auto;
}
.intro-hero {
  padding: 3rem 2rem;
  background:
    radial-gradient(circle at center, rgba(99, 102, 241, 0.15), transparent 60%),
    rgba(15, 23, 42, 0.8);
  backdrop-filter: blur(20px);
  border-radius: 24px;
  border: 1px solid rgba(148, 163, 184, 0.2);
  text-align: center;
  margin-bottom: 3rem;
}
.hero-title {
  font-size: 3rem;
  font-weight: 800;
  margin: 0 0 1.5rem 0;
  color: #f8fafc;
}

/* CARDS */
.tool-card {
  border-radius: 12px;
  border-left: 4px solid;
}
.custom-card {
  border-radius: 12px;
  background: rgba(30, 41, 59, 0.5);
  border-top: 4px solid;
  height: 100%;
}
.border-blue {
  border-color: #3b82f6;
}
.border-green {
  border-color: #22c55e;
}
.shadow-indigo {
  box-shadow: 0 0 15px rgba(99, 102, 241, 0.5);
}
.h-full {
  height: 100%;
}

/* GLOBAL VS LOCAL ANIMATION */
.robot-nav {
  width: 16px;
  height: 16px;
  border-radius: 50%;
  position: absolute;
  top: 180px;
  left: 20px;
  animation: movePath 4s infinite linear;
}
.obstacle-dynamic {
  width: 20px;
  height: 20px;
  border-radius: 4px;
  position: absolute;
  top: 80px;
  left: 140px;
  animation: popObstacle 4s infinite;
}
.local-path-curve {
  position: absolute;
  width: 100px;
  height: 60px;
  border-top: 2px solid #4ade80;
  border-radius: 50%;
  top: 100px;
  left: 60px;
  transform: rotate(-20deg);
  opacity: 0;
  animation: showLocal 4s infinite;
}
@keyframes movePath {
  0% {
    top: 180px;
    left: 20px;
  }
  40% {
    top: 100px;
    left: 100px;
  } /* Encounter Obstacle */
  60% {
    top: 80px;
    left: 180px;
  } /* Dodge */
  100% {
    top: 20px;
    left: 280px;
  }
}
@keyframes popObstacle {
  0%,
  20% {
    transform: scale(0);
  }
  25%,
  100% {
    transform: scale(1);
  }
}
@keyframes showLocal {
  30% {
    opacity: 0;
  }
  40% {
    opacity: 1;
  } /* Show dodge curve */
  60% {
    opacity: 0;
  }
}

/* LAYER STACK */
.layer-stack {
  width: 250px;
  padding: 10px;
  border-radius: 8px;
  text-align: center;
  font-weight: bold;
  margin: 5px 0;
}
.layer-arrow {
  font-size: 1.5rem;
  line-height: 1;
  opacity: 0.5;
}

/* INFLATION ANIMATION */
.obstacle-center {
  width: 40px;
  height: 40px;
  border-radius: 4px;
  z-index: 5;
}
.circle-layer {
  border-radius: 50%;
  position: absolute;
}
.layer-lethal {
  width: 60px;
  height: 60px;
  background: rgba(255, 0, 0, 0.3);
  border: 2px solid red;
  animation: pulseLethal 2s infinite;
}
.layer-inflation {
  width: 140px;
  height: 140px;
  background: radial-gradient(circle, rgba(99, 102, 241, 0.4) 0%, rgba(99, 102, 241, 0) 70%);
}
@keyframes pulseLethal {
  50% {
    transform: scale(1.1);
  }
}

/* BT DIAGRAM */
.bt-node {
  padding: 8px 12px;
  border-radius: 6px;
  font-size: 0.8rem;
  font-family: monospace;
}
.bt-line {
  width: 2px;
  height: 20px;
  background: #64748b;
  margin: 0 auto;
}
.bt-line-red {
  width: 2px;
  height: 20px;
  background: #f87171;
  margin: 0 auto;
}

/* WOBBLY ROBOT */
.path-wobble {
  stroke-dasharray: 10;
  animation: dashMove 1s infinite linear;
}
.robot-wobble {
  width: 15px;
  height: 15px;
  border-radius: 50%;
  position: absolute;
  offset-path: path('M 10 140 Q 50 10, 100 140 T 190 10');
  animation: followWobble 4s infinite linear;
}
@keyframes followWobble {
  0% {
    offset-distance: 0%;
  }
  100% {
    offset-distance: 100%;
  }
}
@keyframes dashMove {
  to {
    stroke-dashoffset: -20;
  }
}

/* UTILS */
.bg-slate-900 {
  background: #0f172a;
}
.bg-slate-800 {
  background: #1e293b;
}
.text-xxs {
  font-size: 0.7rem;
}
.font-mono {
  font-family: 'Fira Code', monospace;
}
</style>
