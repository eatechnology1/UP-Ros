<template>
  <div
    class="fullscreen bg-slate text-center q-pa-md flex flex-center relative-position overflow-hidden"
  >
    <!-- FONDO ANIMADO (Círculos difusos) -->
    <div class="bg-shape shape-1"></div>
    <div class="bg-shape shape-2"></div>

    <div class="z-top content-container column items-center">
      <!-- 1. EL ROBOT PERDIDO (NUEVO) -->
      <div class="robot-wrapper q-mb-lg">
        <!-- El 404 ahora está detrás del robot como un elemento gráfico -->
        <div class="error-code-bg">404</div>

        <img
          src="~assets/images/404.webp"
          alt="Robot Confundido"
          class="robot-image floating-robot"
        />
        <div class="robot-shadow"></div>
      </div>

      <!-- 2. MENSAJE TEMÁTICO -->
      <div class="text-h4 text-primary text-weight-bold q-mb-md glitch-text">
        Transform Error: Frame [Page] does not exist
      </div>

      <p class="text-subtitle1 q-mb-lg description-text page-description">
        Parece que el <strong>TF Tree</strong> se ha roto. No podemos encontrar la transformación
        entre <code>/world</code> y la página que buscas. <br />
        Probablemente el nodo se ha caído o el tópico no está publicado.
      </p>

      <!-- 3. BOTONES DE ACCIÓN -->
      <div class="row justify-center q-gutter-md q-mb-xl">
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

      <!-- 4. TERMINAL DECORATIVA (Más compacta) -->
      <div class="terminal-mockup">
        <div class="terminal-header">
          <div class="dot red"></div>
          <div class="dot yellow"></div>
          <div class="dot green"></div>
          <div class="terminal-title q-ml-sm" style="font-size: 10px; color: var(--text-muted)">
            ros2_launch_log.txt
          </div>
        </div>
        <div class="terminal-body">
          <span class="text-red"
            >[ERROR] [171562.332]: Lookup would require extrapolation into the future.</span
          ><br />
          <span style="color: var(--text-muted)"
            >[INFO] [171562.335]: Waiting for service /get_page to be available...</span
          ><br />
          <span class="text-warning"
            >[WARN] [171562.450]: Destination unreachable. Robot is lost in space.</span
          >
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
  background-color: var(--bg-page);
}

/* --- ROBOT & 404 --- */
.robot-wrapper {
  position: relative;
  width: 100%;
  max-width: 400px;
  height: 300px; /* Ajusta según la proporción de tu imagen */
  display: flex;
  justify-content: center;
  align-items: center;
}

.robot-image {
  width: 280px;
  height: auto;
  z-index: 2;
  filter: drop-shadow(0 10px 20px rgba(0, 0, 0, 0.3));
}

.floating-robot {
  animation: float-robot 6s ease-in-out infinite;
}

.robot-shadow {
  position: absolute;
  bottom: 20px;
  width: 150px;
  height: 20px;
  background: var(--bg-surface-hover);
  border-radius: 50%;
  filter: blur(10px);
  animation: shadow-scale 6s ease-in-out infinite;
  z-index: 1;
}

.error-code-bg {
  position: absolute;
  font-size: 12rem;
  font-weight: 900;
  color: var(--border-subtle);
  z-index: 0;
  top: 50%;
  left: 50%;
  transform: translate(-50%, -50%);
  user-select: none;
  font-family: 'Fira Code', monospace;
}

@keyframes float-robot {
  0%,
  100% {
    transform: translateY(0);
  }
  50% {
    transform: translateY(-20px);
  }
}

@keyframes shadow-scale {
  0%,
  100% {
    transform: scale(1);
    opacity: 0.3;
  }
  50% {
    transform: scale(0.8);
    opacity: 0.1;
  }
}

/* --- FONDOS --- */
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
  background: #38bdf8;
  animation: float 10s infinite alternate;
}

.shape-2 {
  bottom: -10%;
  right: -10%;
  width: 600px;
  height: 600px;
  background: #6366f1;
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

/* --- CONTENIDO --- */
.content-container {
  position: relative;
  z-index: 10;
  max-width: 800px;
}

.description-text {
  max-width: 600px;
  margin-inline: auto;
  line-height: 1.6;
}

.page-description {
  color: var(--text-secondary);
}

/* --- BOTONES --- */
.tech-btn {
  background: #38bdf8;
  color: #0f172a;
  font-weight: 700;
  border-radius: 8px;
  padding: 10px 24px;
  transition: all 0.3s ease;
}

.tech-btn:hover {
  box-shadow: 0 0 20px rgba(56, 189, 248, 0.4);
  transform: translateY(-2px);
}

.tech-btn-outline {
  color: var(--text-muted);
  border-color: var(--border-medium);
  font-weight: 600;
  border-radius: 8px;
  padding: 10px 24px;
}

.tech-btn-outline:hover {
  color: var(--text-primary);
  border-color: var(--text-secondary);
  background: var(--bg-surface-hover);
}

/* --- TERMINAL --- */
.terminal-mockup {
  width: 100%;
  max-width: 650px;
  background: var(--bg-surface-solid);
  border-radius: 12px;
  border: 1px solid var(--border-subtle);
  box-shadow: 0 20px 50px var(--shadow-md);
  text-align: left;
  overflow: hidden;
  font-size: 0.8rem;
}

.terminal-header {
  background: var(--bg-deep);
  padding: 10px 16px;
  display: flex;
  align-items: center;
  gap: 8px;
  border-bottom: 1px solid var(--border-subtle);
}

.dot {
  width: 10px;
  height: 10px;
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
  padding: 16px;
  font-family: 'Fira Code', monospace;
  line-height: 1.6;
}
</style>
