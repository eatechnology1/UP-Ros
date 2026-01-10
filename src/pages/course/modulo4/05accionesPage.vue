<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-purple-4 text-weight-bold q-mb-sm">
          MÓDULO 4.5: MISIONES COMPLEJAS
        </div>

        <h1 class="hero-title">Acciones <span class="text-white">(Actions)</span></h1>

        <TextBlock>
          Los Servicios son geniales para cosas rápidas ("enciende la luz"), pero terribles para
          tareas largas ("navega a la cocina"). Si usas un Servicio para mover el robot, te quedarás
          bloqueado varios minutos sin saber qué pasa [web:6]. Las
          <strong>Acciones</strong> resuelven esto añadiendo <strong>Feedback</strong> en tiempo
          real y un botón de <strong>Cancelar</strong> [web:6][web:8].
        </TextBlock>
      </div>
    </section>

    <!-- 2. ANATOMÍA DE UNA ACCIÓN (LA TRÍADA) -->
    <div class="section-group self-stretch">
      <SectionTitle>1. La Tríada de Comunicación</SectionTitle>

      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-5">
          <TextBlock>
            Una Acción no es un protocolo nuevo; es un "truco" inteligente que combina 3 canales
            estándar bajo el capó [web:6][web:3].
            <br /><br />
            <ol class="q-pl-md q-mt-sm text-grey-4 tool-list">
              <li>
                🎯 <strong>Goal (Servicio):</strong> El Cliente pide la misión. El Servidor acepta o
                rechaza.
              </li>
              <li>
                📢 <strong>Feedback (Tópico):</strong> El Servidor reporta el progreso ("Voy por el
                50%").
              </li>
              <li>
                🏁 <strong>Result (Servicio):</strong> El Servidor notifica que terminó (o que
                falló).
              </li>
            </ol>
          </TextBlock>
        </div>

        <div class="col-12 col-md-7">
          <!-- VISUAL ACTION VIZ -->
          <div
            class="tool-card action-viz q-pa-lg bg-black relative-position overflow-hidden shadow-2"
          >
            <div
              class="row items-center justify-between relative-position full-height"
              style="z-index: 2"
            >
              <!-- CLIENT -->
              <div class="column items-center" style="width: 90px">
                <div class="node-box bg-green-9 shadow-green border-light transition-hover">
                  <q-icon name="person_pin" color="white" size="2rem" />
                </div>
                <div class="text-weight-bold text-green-4 q-mt-sm font-mono text-xs">Client</div>
                <div class="text-caption text-grey-4 text-center q-mt-xs text-xs">
                  "Ve a la cocina"
                </div>
              </div>

              <!-- CHANNELS ZONE -->
              <div
                class="col relative-position column justify-center q-mx-md"
                style="height: 190px"
              >
                <!-- 1. GOAL -->
                <div class="channel-row relative-position">
                  <div class="line-bg"></div>
                  <div
                    class="packet-goal bg-blue-5 text-black text-xxs text-weight-bold rounded-borders q-px-xs"
                  >
                    GOAL
                  </div>
                  <div class="text-caption text-blue-3 absolute-right label-right">1. Petición</div>
                </div>

                <!-- 2. FEEDBACK -->
                <div class="channel-row relative-position q-my-md">
                  <div class="line-bg"></div>
                  <div
                    class="packet-fb bg-purple-5 text-black text-xxs text-weight-bold rounded-borders q-px-xs"
                  >
                    FB
                  </div>
                  <div
                    class="packet-fb bg-purple-5 text-black text-xxs text-weight-bold rounded-borders q-px-xs"
                    style="animation-delay: 1.2s"
                  >
                    FB
                  </div>
                  <div class="text-caption text-purple-3 absolute-left label-left">
                    2. Progreso...
                  </div>
                </div>

                <!-- 3. RESULT -->
                <div class="channel-row relative-position">
                  <div class="line-bg"></div>
                  <div
                    class="packet-res bg-yellow-5 text-black text-xxs text-weight-bold rounded-borders q-px-xs"
                  >
                    RES
                  </div>
                  <div class="text-caption text-yellow-3 absolute-right label-right">3. Final</div>
                </div>
              </div>

              <!-- SERVER -->
              <div class="column items-center" style="width: 90px">
                <div class="node-box bg-red-9 shadow-red border-light transition-hover">
                  <q-icon name="smart_toy" color="white" size="2rem" />
                </div>
                <div class="text-weight-bold text-red-4 q-mt-sm font-mono text-xs">Server</div>
                <div class="text-caption text-grey-4 text-center q-mt-xs text-xs">Navegando...</div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. INTERFAZ .ACTION (LA DEFINICIÓN) -->
    <div class="section-group self-stretch">
      <SectionTitle>2. El Contrato Triple (.action)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Para definir una acción, se usa un archivo <code>.action</code> con tres secciones:
            Goal, Result y Feedback [web:6][web:3]. <br /><br />
            A diferencia de los servicios (un separador <code>---</code>), las acciones usan
            <strong>dos separadores</strong> para dividir: 1. Meta (Goal), 2. Resultado (Result), 3.
            Progreso (Feedback) [web:3].
          </TextBlock>
        </template>

        <template #right>
          <div
            class="tool-card action-file-card bg-slate-900 q-pa-none border-purple shadow-2 overflow-hidden"
          >
            <div
              class="row justify-between items-center q-px-md q-py-sm bg-black border-bottom-dark"
            >
              <span class="text-grey-5 font-mono text-xs"
                >my_robot_interfaces/action/Navigate.action</span
              >
              <q-icon name="rocket_launch" color="purple-4" size="xs" />
            </div>

            <div class="code-structure font-mono text-sm q-pa-md bg-slate-800">
              <div class="text-caption text-grey-6 text-xxs q-mb-xs"># 1. GOAL (¿A dónde voy?)</div>
              <div class="row q-mb-xs">
                <div class="col-4 text-blue-4">float32</div>
                <div class="col text-white">x</div>
              </div>
              <div class="row q-mb-sm">
                <div class="col-4 text-blue-4">float32</div>
                <div class="col text-white">y</div>
              </div>

              <div class="separator-dashed q-my-sm">---</div>

              <div class="text-caption text-grey-6 text-xxs q-mb-xs">
                # 2. RESULT (¿Llegué bien?)
              </div>
              <div class="row q-mb-xs">
                <div class="col-4 text-blue-4">bool</div>
                <div class="col text-white">success</div>
              </div>
              <div class="row q-mb-sm">
                <div class="col-4 text-blue-4">string</div>
                <div class="col text-white">message</div>
              </div>

              <div class="separator-dashed q-my-sm">---</div>

              <div class="text-caption text-grey-6 text-xxs q-mb-xs">
                # 3. FEEDBACK (¿Cuánto falta?)
              </div>
              <div class="row">
                <div class="col-4 text-blue-4">float32</div>
                <div class="col text-white">distance_remaining</div>
              </div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 4. LA MÁQUINA DE ESTADOS (EL CEREBRO) -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>3. El Cerebro del Servidor</SectionTitle>
      <AlertBlock type="info" title="No es solo 'On/Off'">
        Una Acción es una mini máquina de estados. El servidor puede rechazar tu meta si está
        ocupado, o tú puedes cancelarla si te arrepientes [web:6][web:8].
      </AlertBlock>

      <div class="row q-mt-md justify-center">
        <div class="col-12 col-md-10">
          <div class="tool-card state-machine bg-slate-800 q-pa-lg text-center shadow-2">
            <div class="row items-center justify-around">
              <!-- STATE 1 -->
              <div class="state-circle bg-grey-9 border-grey text-grey-5">IDLE</div>
              <q-icon name="arrow_forward" color="grey-6" />

              <!-- STATE 2 -->
              <div class="state-circle bg-blue-9 border-blue text-white pulse-anim">EXECUTING</div>

              <div class="column q-gutter-sm">
                <div class="row items-center">
                  <q-icon name="arrow_forward" color="green-4" />
                  <div class="state-pill bg-green-9 text-xs q-ml-sm text-green-3">SUCCEEDED</div>
                </div>
                <div class="row items-center">
                  <q-icon name="arrow_forward" color="red-4" />
                  <div class="state-pill bg-red-9 text-xs q-ml-sm text-red-3">ABORTED</div>
                </div>
                <div class="row items-center">
                  <q-icon name="arrow_forward" color="orange-4" />
                  <div class="state-pill bg-orange-9 text-xs q-ml-sm text-orange-3">CANCELED</div>
                </div>
              </div>
            </div>

            <div class="text-caption text-grey-5 q-mt-md">
              El servidor puede abortar si encuentra un obstáculo, o tú puedes cancelar si pulsas
              "Stop" [web:6][web:9].
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 5. HERRAMIENTAS CLI -->
    <div class="section-group self-stretch q-mt-xl q-mb-xl">
      <SectionTitle>4. Acciones en la Terminal</SectionTitle>
      <TextBlock>
        Probar acciones a mano es vital para asegurarte de que tu servidor funciona antes de
        escribir el cliente en Python/C++ [web:6][web:7]. Nota el flag <code>--feedback</code> para
        ver la actualización en vivo [web:10].
      </TextBlock>

      <div class="tool-card cli-card bg-black q-pa-md q-mt-md border-purple-glow shadow-2">
        <div class="window-dots row q-mb-md">
          <div class="dot bg-red-5 q-mr-xs"></div>
          <div class="dot bg-yellow-5 q-mr-xs"></div>
          <div class="dot bg-green-5"></div>
        </div>

        <div class="text-grey-5 font-mono text-xs q-mb-sm"># Enviar una meta y ver el feedback</div>

        <div class="cmd-line q-pa-sm rounded bg-slate-900 q-mb-md">
          <span class="text-purple-4 font-mono text-sm text-break">
            $ ros2 action send_goal /navigate my_pkg/action/Navigate "{x: 5.0, y: 0.0}" --feedback
          </span>
        </div>

        <div class="console-output q-mt-sm font-mono text-xs">
          <div class="text-blue-3">Waiting for an action server to become available...</div>
          <div class="text-green-4">Goal accepted with ID: a1b2...</div>
          <br />
          <div class="text-purple-3">Feedback:</div>
          <div class="text-grey-4">distance_remaining: 5.0</div>
          <div class="text-grey-4">distance_remaining: 4.5</div>
          <div class="text-grey-4">distance_remaining: 4.0 ...</div>
          <br />
          <div class="text-yellow-4">Result:</div>
          <div class="text-white">success: true</div>
          <div class="text-white">message: "Llegué al destino"</div>
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
</script>

<style scoped>
/* --- ESTILOS MAESTROS --- */
.intro-hero,
.section-group {
  width: 100%;
  max-width: 1100px;
  margin: 0 auto 3.5rem auto;
}

.intro-hero {
  padding: 3rem 2rem;
  background:
    radial-gradient(circle at center, rgba(168, 85, 247, 0.15), transparent 60%),
    rgba(15, 23, 42, 0.8);
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

/* TOOL CARDS */
.tool-card {
  height: 100%;
  border-radius: 16px;
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(255, 255, 255, 0.05);
}

/* ACTION VIZ */
.action-viz {
  border-top: 4px solid #a855f7;
  height: 300px;
}
.node-box {
  width: 70px;
  height: 70px;
  border-radius: 14px;
  display: flex;
  align-items: center;
  justify-content: center;
}
.shadow-green {
  box-shadow: 0 0 15px rgba(74, 222, 128, 0.4);
}
.shadow-red {
  box-shadow: 0 0 15px rgba(248, 113, 113, 0.4);
}

.channel-row {
  height: 46px;
  width: 100%;
  display: flex;
  align-items: center;
  position: relative;
}
.line-bg {
  height: 2px;
  width: 100%;
  background: #334155;
}

.label-right {
  position: absolute;
  right: 0;
  top: -18px;
}
.label-left {
  position: absolute;
  left: 0;
  top: -18px;
}

/* Packet Animations (ciclo Goal -> Feedback -> Result) */
.packet-goal,
.packet-fb,
.packet-res {
  position: absolute;
  top: 8px;
  border-radius: 4px;
  padding: 2px 6px;
  box-shadow: 0 0 10px rgba(255, 255, 255, 0.2);
  opacity: 0;
}

/* 0–25% Goal, 30–70% Feedback, 75–100% Result */
@keyframes goalTravel {
  0% {
    left: 0%;
    opacity: 0;
    transform: scale(0.5);
  }
  10% {
    opacity: 1;
    transform: scale(1);
  }
  25% {
    left: 85%;
    opacity: 1;
  }
  30% {
    left: 90%;
    opacity: 0;
    transform: scale(0.5);
  }
  100% {
    left: 90%;
    opacity: 0;
  }
}
@keyframes fbTravel {
  0% {
    right: 0%;
    opacity: 0;
  }
  30% {
    right: 0%;
    opacity: 0;
  }
  40% {
    right: 15%;
    opacity: 1;
  }
  60% {
    right: 80%;
    opacity: 1;
  }
  70% {
    right: 90%;
    opacity: 0;
  }
  100% {
    right: 90%;
    opacity: 0;
  }
}
@keyframes resTravel {
  0% {
    right: 0%;
    opacity: 0;
  }
  75% {
    right: 0%;
    opacity: 0;
  }
  82% {
    right: 20%;
    opacity: 1;
  }
  95% {
    right: 80%;
    opacity: 1;
  }
  100% {
    right: 90%;
    opacity: 0;
  }
}

.packet-goal {
  animation: goalTravel 4s infinite linear;
}
.packet-fb {
  animation: fbTravel 4s infinite linear;
}
.packet-res {
  animation: resTravel 4s infinite linear;
}

/* ACTION FILE CARD */
.action-file-card {
  border-left: 4px solid #a855f7;
}
.separator-dashed {
  border-bottom: 2px dashed #64748b;
  width: 100%;
  text-align: center;
  line-height: 0.1em;
  color: #64748b;
}

/* STATE MACHINE */
.state-machine {
  border-radius: 16px;
}
.state-circle {
  width: 80px;
  height: 80px;
  border-radius: 50%;
  border: 3px solid;
  display: flex;
  align-items: center;
  justify-content: center;
  font-weight: bold;
  font-size: 0.8rem;
}
.pulse-anim {
  animation: pulse 2.5s infinite;
}
@keyframes pulse {
  0% {
    box-shadow: 0 0 0 0 rgba(59, 130, 246, 0.7);
  }
  70% {
    box-shadow: 0 0 0 12px rgba(59, 130, 246, 0);
  }
  100% {
    box-shadow: 0 0 0 0 rgba(59, 130, 246, 0);
  }
}
.state-pill {
  padding: 4px 8px;
  border-radius: 999px;
  font-weight: bold;
}
.border-grey {
  border-color: #4b5563;
}
.border-blue {
  border-color: #3b82f6;
}

/* CLI CARD */
.border-purple-glow {
  border: 1px solid #a855f7;
  box-shadow: 0 0 18px rgba(168, 85, 247, 0.2);
}
.window-dots .dot {
  width: 10px;
  height: 10px;
  border-radius: 50%;
}
.cmd-line {
  border-radius: 4px;
}
.console-output {
  background: #020617;
  border-radius: 8px;
  padding: 10px;
}

/* UTILS */
.font-mono {
  font-family: 'Fira Code', monospace;
}
.text-xxs {
  font-size: 0.7rem;
}
.text-xs {
  font-size: 0.8rem;
}
.text-sm {
  font-size: 0.9rem;
}
.bg-slate-900 {
  background: #0f172a;
}
.bg-slate-800 {
  background: #1e293b;
}
.border-light {
  border: 1px solid rgba(255, 255, 255, 0.1);
}
.border-bottom-dark {
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}
.tool-list {
  list-style: none;
  padding: 0;
}
.tool-list li {
  margin-bottom: 10px;
  font-size: 0.95rem;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
