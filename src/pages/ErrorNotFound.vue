<template>
  <div
    class="fullscreen bg-slate text-white text-center q-pa-md flex flex-center relative-position overflow-hidden"
  >
    <!-- FONDO ANIMADO (Círculos difusos) -->
    <div class="bg-shape shape-1"></div>
    <div class="bg-shape shape-2"></div>

    <div class="z-top content-container">
      <!-- CÓDIGO DE ERROR GIGANTE -->
      <div class="error-code">404</div>

      <!-- MENSAJE TEMÁTICO DE ROBÓTICA -->
      <div class="text-h4 text-primary text-weight-bold q-mb-md">
        Transform Error: Frame [Page] does not exist
      </div>

      <p class="text-grey-4 text-subtitle1 q-mb-xl" style="max-width: 600px; margin-inline: auto">
        Parece que el <strong>TF Tree</strong> se ha roto. No podemos encontrar la transformación
        entre <code>/world</code> y la página que buscas. <br /><br />
        Probablemente el nodo se ha caído o el tópico no está publicado.
      </p>

      <!-- BOTONES DE ACCIÓN -->
      <div class="row justify-center q-gutter-md">
        <q-btn
          class="tech-btn"
          unelevated
          to="/"
          label="Reiniciar Sistema (Home)"
          icon="restart_alt"
          no-caps
        />
        <q-btn
          class="tech-btn-outline"
          outline
          @click="$router.go(-1)"
          label="Volver Atrás"
          icon="arrow_back"
          no-caps
        />
      </div>

      <!-- TERMINAL DECORATIVA -->
      <div class="terminal-mockup q-mt-xl">
        <div class="terminal-header">
          <div class="dot red"></div>
          <div class="dot yellow"></div>
          <div class="dot green"></div>
        </div>
        <div class="terminal-body">
          <span class="text-red"
            >[ERROR] [171562.332]: Lookup would require extrapolation into the future.</span
          ><br />
          <span class="text-grey"
            >[INFO] [171562.335]: Waiting for service /get_page to be available...</span
          ><br />
          <span class="text-grey">[WARN] [171562.450]: Destination unreachable.</span>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
// No logic needed
</script>

<style scoped>
.bg-slate {
  background-color: #0f172a; /* Slate-900 */
}

/* EFECTOS DE FONDO */
.bg-shape {
  position: absolute;
  border-radius: 50%;
  filter: blur(80px);
  opacity: 0.15;
  z-index: 0;
}

.shape-1 {
  top: -10%;
  left: -10%;
  width: 500px;
  height: 500px;
  background: #38bdf8; /* Sky-400 */
  animation: float 10s infinite alternate;
}

.shape-2 {
  bottom: -10%;
  right: -10%;
  width: 600px;
  height: 600px;
  background: #6366f1; /* Indigo-500 */
  animation: float 15s infinite alternate-reverse;
}

@keyframes float {
  0% {
    transform: translate(0, 0);
  }
  100% {
    transform: translate(30px, 30px);
  }
}

/* TIPOGRAFÍA */
.error-code {
  font-size: 10rem;
  font-weight: 900;
  line-height: 1;
  background: linear-gradient(135deg, #f1f5f9 0%, #94a3b8 100%);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  opacity: 0.1;
  user-select: none;
  margin-bottom: -2rem; /* Superposición visual */
}

.content-container {
  position: relative;
  z-index: 10;
}

/* BOTONES */
.tech-btn {
  background: #38bdf8;
  color: #0f172a;
  font-weight: 700;
  border-radius: 8px;
  padding: 10px 24px;
  transition: all 0.3s ease;
}

.tech-btn:hover {
  box-shadow: 0 0 15px rgba(56, 189, 248, 0.4);
  transform: translateY(-2px);
}

.tech-btn-outline {
  color: #94a3b8;
  border-color: #475569;
  font-weight: 600;
  border-radius: 8px;
  padding: 10px 24px;
}

.tech-btn-outline:hover {
  color: #f1f5f9;
  border-color: #cbd5e1;
  background: rgba(255, 255, 255, 0.05);
}

/* TERMINAL FAKE */
.terminal-mockup {
  max-width: 600px;
  margin: 40px auto 0;
  background: #1e293b; /* Slate-800 */
  border-radius: 12px;
  border: 1px solid rgba(255, 255, 255, 0.1);
  box-shadow: 0 20px 50px rgba(0, 0, 0, 0.3);
  text-align: left;
  overflow: hidden;
}

.terminal-header {
  background: #0f172a;
  padding: 12px 16px;
  display: flex;
  gap: 8px;
}

.dot {
  width: 12px;
  height: 12px;
  border-radius: 50%;
}
.red {
  background: #ef4444;
}
.yellow {
  background: #f59e0b;
}
.green {
  background: #22c55e;
}

.terminal-body {
  padding: 20px;
  font-family: 'Fira Code', monospace;
  font-size: 0.85rem;
  line-height: 1.6;
}
</style>
