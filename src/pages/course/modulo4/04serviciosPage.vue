<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-yellow-4 text-weight-bold q-mb-sm">
          MÓDULO 4.4: PETICIÓN Y RESPUESTA
        </div>

        <h1 class="hero-title">Servicios <span class="text-white">(Services)</span></h1>

        <TextBlock>
          Los Tópicos son excelentes para datos continuos, pero pésimos para lógica de control. Si
          quieres preguntarle al robot "¿Estás listo?" o ordenarle "Toma una foto ahora", necesitas
          una confirmación inmediata. Los <strong>Servicios</strong> son comunicaciones síncronas
          diseñadas para transacciones puntuales.
        </TextBlock>
      </div>
    </section>

    <!-- 2. ANATOMÍA CLIENTE/SERVIDOR -->
    <div class="section-group self-stretch">
      <SectionTitle>1. El Patrón de Solicitud</SectionTitle>

      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-5">
          <TextBlock>
            A diferencia de los Tópicos (1-a-N), los Servicios suelen ser <strong>1-a-1</strong>.
            <br /><br />
            <strong>El Bloqueo:</strong> Normalmente, el Cliente se "congela" (espera) hasta que el
            Servidor responde.
          </TextBlock>
          <ul class="q-pl-md q-mt-md text-grey-4 tool-list">
            <li>
              🛎️ <strong>Cliente (Client):</strong> Inicia la conversación. Envía una
              <em>Request</em>.
            </li>
            <li>
              👨‍🍳 <strong>Servidor (Server):</strong> Espera peticiones. Procesa la lógica y devuelve
              una <em>Response</em>.
            </li>
          </ul>
        </div>

        <div class="col-12 col-md-7">
          <!-- VISUAL ANIMATION REQ/RES -->
          <div
            class="tool-card service-viz q-pa-lg bg-black relative-position overflow-hidden shadow-2"
          >
            <div
              class="row items-center justify-between relative-position full-height"
              style="z-index: 2"
            >
              <!-- CLIENT -->
              <div class="column items-center z-top">
                <div class="node-box bg-purple-9 shadow-purple border-light transition-hover">
                  <q-icon name="person" color="white" size="2rem" />
                </div>
                <div class="text-weight-bold text-purple-4 q-mt-sm font-mono text-xs">Client</div>
                <!-- Dynamic Status Text -->
                <div class="status-badge-client text-xxs q-mt-xs font-mono"></div>
              </div>

              <!-- INTERACTION ZONE -->
              <div class="col relative-position text-center q-px-md" style="height: 120px">
                <!-- SERVICE NAME -->
                <div class="absolute-center" style="z-index: 10; top: 50%">
                  <div
                    class="bg-slate-900 q-px-sm q-py-xs rounded-borders text-yellow-4 font-mono text-xs inline-block shadow-1 border-yellow-dim"
                  >
                    /spawn_turtle
                  </div>
                </div>

                <!-- REQUEST PACKET (Top) -->
                <div class="packet-container absolute" style="top: 25%; width: 100%; left: 0">
                  <div class="arrow-path bg-grey-9"></div>
                  <div class="packet-req">
                    <span class="text-xxs text-black text-weight-bold">REQ</span>
                  </div>
                </div>

                <!-- RESPONSE PACKET (Bottom) -->
                <div class="packet-container absolute" style="bottom: 25%; width: 100%; left: 0">
                  <div class="arrow-path bg-grey-9"></div>
                  <div class="packet-res">
                    <span class="text-xxs text-black text-weight-bold">RES</span>
                  </div>
                </div>
              </div>

              <!-- SERVER -->
              <div class="column items-center z-top">
                <div
                  class="node-box bg-blue-9 shadow-blue border-light transition-hover server-pulse"
                >
                  <q-icon name="restaurant_menu" color="white" size="2rem" />
                </div>
                <div class="text-weight-bold text-blue-4 q-mt-sm font-mono text-xs">Server</div>
                <div class="status-badge-server text-xxs q-mt-xs font-mono"></div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. LA INTERFAZ .SRV (EL CONTRATO) -->
    <div class="section-group self-stretch">
      <SectionTitle>2. El Contrato (.srv)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Igual que los mensajes (`.msg`), los servicios necesitan definir qué datos viajan. Pero
            aquí la estructura es doble: <strong>Qué entra</strong> y <strong>Qué sale</strong>.
            <br /><br />
            Los archivos `.srv` se dividen por tres guiones medios <code>---</code>.
            <br />
            Arriba: Request. Abajo: Response.
          </TextBlock>
        </template>

        <template #right>
          <div
            class="tool-card srv-card bg-slate-900 q-pa-none border-yellow shadow-2 overflow-hidden"
          >
            <div
              class="row justify-between items-center q-px-md q-py-sm bg-black border-bottom-dark"
            >
              <span class="text-grey-5 font-mono text-xs">example_interfaces/srv/AddTwoInts</span>
              <q-icon name="description" color="yellow-4" size="xs" />
            </div>

            <div class="code-structure font-mono text-sm q-pa-md bg-slate-800">
              <div class="text-caption text-grey-6 q-mb-xs font-italic text-xxs">
                # REQUEST (Lo que envías)
              </div>

              <div class="row q-mb-xs">
                <div class="col-4 text-blue-4">int64</div>
                <div class="col text-white">a</div>
              </div>
              <div class="row">
                <div class="col-4 text-blue-4">int64</div>
                <div class="col text-white">b</div>
              </div>

              <!-- THE SEPARATOR -->
              <div class="separator-dashed q-my-md text-grey-6">---</div>

              <div class="text-caption text-grey-6 q-mb-xs font-italic text-xxs">
                # RESPONSE (Lo que recibes)
              </div>
              <div class="row">
                <div class="col-4 text-blue-4">int64</div>
                <div class="col text-white">sum</div>
              </div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 4. SINCRONO VS ASINCRONO (EL PELIGRO) -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>3. El Peligro del Bloqueo (Deadlock)</SectionTitle>
      <AlertBlock type="danger" title="¡Nunca bloquees el hilo principal!">
        Si llamas a un servicio de forma <strong>Síncrona</strong> dentro de un callback, tu robot
        se congelará. El nodo deja de procesar sensores mientras espera la respuesta. Si el servidor
        tarda 5 segundos, tu robot está ciego 5 segundos.
      </AlertBlock>

      <div class="row q-col-gutter-lg q-mt-sm">
        <div class="col-12 col-md-6">
          <div class="tool-card bad-practice bg-slate-900 q-pa-lg border-red shadow-2">
            <div class="row items-center q-mb-md">
              <q-icon name="cancel" color="red-4" size="md" class="q-mr-sm" />
              <div class="text-red-4 text-subtitle1 text-weight-bold">Llamada Síncrona</div>
            </div>
            <p class="text-caption text-grey-4">
              "Camarero, tráeme la sopa. <strong>No me moveré ni respiraré</strong> hasta que
              vuelvas".
            </p>
            <div
              class="q-mt-md bg-black q-pa-md rounded-borders font-mono text-xs text-red-3 border-light"
            >
              future = client.call(req) <br />
              # ❌ EL ROBOT SE DETIENE AQUÍ
            </div>
          </div>
        </div>
        <div class="col-12 col-md-6">
          <div class="tool-card good-practice bg-slate-900 q-pa-lg border-green shadow-2">
            <div class="row items-center q-mb-md">
              <q-icon name="check_circle" color="green-4" size="md" class="q-mr-sm" />
              <div class="text-green-4 text-subtitle1 text-weight-bold">Llamada Asíncrona</div>
            </div>
            <p class="text-caption text-grey-4">
              "Camarero, encargo la sopa. <strong>Seguiré leyendo el periódico</strong>. Avísame
              cuando esté lista".
            </p>
            <div
              class="q-mt-md bg-black q-pa-md rounded-borders font-mono text-xs text-green-3 border-light"
            >
              future = client.call_async(req)<br />
              # ✅ EL ROBOT SIGUE VIVO
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 5. HERRAMIENTAS CLI -->
    <div class="section-group self-stretch q-mt-xl q-mb-xl">
      <SectionTitle>4. Llamando Servicios desde Terminal</SectionTitle>
      <TextBlock>
        A veces necesitas resetear una simulación o calibrar un sensor manualmente. No escribas
        código para eso, usa la terminal.
      </TextBlock>

      <div
        class="tool-card cli-card bg-black q-pa-md q-mt-lg border-yellow-glow shadow-glow-yellow"
      >
        <!-- Terminal Header -->
        <div class="window-dots row q-mb-md">
          <div class="dot bg-red-5 q-mr-xs"></div>
          <div class="dot bg-yellow-5 q-mr-xs"></div>
          <div class="dot bg-green-5"></div>
        </div>

        <div class="text-grey-6 font-mono text-xs q-mb-sm">
          # Sintaxis: ros2 service call &lt;nombre&gt; &lt;tipo&gt; &lt;datos&gt;
        </div>

        <div class="cmd-line q-pa-sm rounded bg-slate-900 q-mb-md">
          <span class="text-green-4 font-mono text-sm text-break">
            $ ros2 service call /spawn_turtle turtlesim/srv/Spawn "{x: 2.0, y: 2.0, name:
            'tortuga_ninja'}"
          </span>
        </div>

        <div class="text-white font-mono text-xs q-pl-sm border-left-yellow">
          <div class="text-grey-5">
            requester: making request: turtlesim.srv.Spawn_Request(x=2.0, y=2.0, theta=0.0,
            name='tortuga_ninja')
          </div>
          <br />
          <div class="text-yellow-4">response:</div>
          <div>turtlesim.srv.Spawn_Response(name='tortuga_ninja')</div>
        </div>

        <div class="text-caption text-yellow-2 q-mt-md text-right font-italic opacity-80">
          "¡Tortuga creada manualmente!"
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
    radial-gradient(circle at center, rgba(250, 204, 21, 0.15), transparent 60%),
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

/* SERVICE VIZ & ANIMATION */
.service-viz {
  border-top: 4px solid #facc15;
  height: 320px;
}
.full-height {
  height: 100%;
}
.z-top {
  z-index: 5;
}

.node-box {
  width: 70px;
  height: 70px;
  border-radius: 14px;
  display: flex;
  align-items: center;
  justify-content: center;
}

.arrow-path {
  height: 2px;
  width: 100%;
  border-radius: 1px;
  opacity: 0.3;
}

/* Packet Animations */
/* Total Cycle: 4s.
   0-1.5s: Req Travel.
   1.5-2.5s: Server Process.
   2.5-4s: Res Travel */

.packet-req {
  position: absolute;
  top: -10px;
  left: 0;
  background: #facc15;
  padding: 2px 6px;
  border-radius: 4px;
  box-shadow: 0 0 10px #facc15;
  animation: reqTravel 4s infinite linear;
  opacity: 0;
}

.packet-res {
  position: absolute;
  top: -10px;
  right: 0;
  background: #4ade80;
  padding: 2px 6px;
  border-radius: 4px;
  box-shadow: 0 0 10px #4ade80;
  animation: resTravel 4s infinite linear;
  opacity: 0;
}

@keyframes reqTravel {
  0% {
    left: 10%;
    opacity: 0;
    transform: scale(0.5);
  }
  10% {
    opacity: 1;
    transform: scale(1);
  }
  35% {
    left: 85%;
    opacity: 1;
    transform: scale(1);
  }
  40% {
    left: 90%;
    opacity: 0;
    transform: scale(0.5);
  }
  100% {
    left: 90%;
    opacity: 0;
  }
}

@keyframes resTravel {
  0% {
    right: 90%;
    opacity: 0;
  }
  60% {
    right: 10%;
    opacity: 0;
    transform: scale(0.5);
  } /* Wait */
  70% {
    opacity: 1;
    transform: scale(1);
  }
  95% {
    right: 85%;
    opacity: 1;
    transform: scale(1);
  }
  100% {
    right: 90%;
    opacity: 0;
    transform: scale(0.5);
  }
}

/* Status Text Logic */
.status-badge-client::after {
  content: 'Idle';
  color: #94a3b8;
  animation: clientText 4s infinite;
}
.status-badge-server::after {
  content: 'Listening';
  color: #94a3b8;
  animation: serverText 4s infinite;
}

@keyframes clientText {
  0%,
  35% {
    content: 'Enviando...';
    color: #facc15;
  }
  36%,
  95% {
    content: 'Bloqueado (Waiting)';
    color: #f87171;
  }
  96%,
  100% {
    content: 'Recibido!';
    color: #4ade80;
  }
}

@keyframes serverText {
  0%,
  35% {
    content: 'Listening';
    color: #94a3b8;
    opacity: 0.5;
  }
  36%,
  60% {
    content: 'PROCESANDO...';
    color: #facc15;
    font-weight: bold;
    opacity: 1;
  }
  61%,
  100% {
    content: 'Listening';
    color: #94a3b8;
    opacity: 0.5;
  }
}

.server-pulse {
  animation: pulseProcess 4s infinite;
}
@keyframes pulseProcess {
  0%,
  35% {
    box-shadow: 0 0 15px rgba(59, 130, 246, 0.4);
    transform: scale(1);
  }
  36%,
  60% {
    box-shadow: 0 0 30px #facc15;
    transform: scale(1.1);
    border-color: #facc15;
  }
  61%,
  100% {
    box-shadow: 0 0 15px rgba(59, 130, 246, 0.4);
    transform: scale(1);
  }
}

/* SRV CARD */
.border-yellow {
  border-left: 4px solid #facc15;
}
.separator-dashed {
  border-bottom: 2px dashed #475569;
  width: 100%;
  text-align: center;
  line-height: 0.1em;
  margin: 15px 0 15px 0;
}

/* PRACTICE CARDS */
.bad-practice {
  border-left: 4px solid #ef4444;
}
.good-practice {
  border-left: 4px solid #4ade80;
}
.border-red {
  border-color: rgba(239, 68, 68, 0.3);
}
.border-green {
  border-color: rgba(74, 222, 128, 0.3);
}

/* CLI CARD */
.border-yellow-glow {
  border: 1px solid #facc15;
}
.shadow-glow-yellow {
  box-shadow: 0 0 20px rgba(250, 204, 21, 0.15);
}
.border-left-yellow {
  border-left: 2px solid #facc15;
}
.window-dots .dot {
  width: 10px;
  height: 10px;
  border-radius: 50%;
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
.border-yellow-dim {
  border: 1px solid rgba(250, 204, 21, 0.3);
}
.shadow-purple {
  box-shadow: 0 0 15px rgba(168, 85, 247, 0.4);
}
.shadow-blue {
  box-shadow: 0 0 15px rgba(59, 130, 246, 0.4);
}
.transition-hover {
  transition: transform 0.2s;
}
.tool-list {
  list-style: none;
  padding: 0;
}
.tool-list li {
  margin-bottom: 12px;
  font-size: 1rem;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
