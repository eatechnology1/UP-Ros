<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-accent text-weight-bold q-mb-sm">
          MÓDULO 0: HERRAMIENTAS DE SUPERVIVENCIA
        </div>

        <h1 class="hero-title">Editores de Texto en <span class="text-primary">Terminal</span></h1>

        <TextBlock>
          Cuando la interfaz gráfica falla (y fallará), solo te quedará una pantalla negra y un
          cursor parpadeante. Saber editar archivos de configuración en ese abismo es lo que separa
          a los aficionados de los profesionales.
        </TextBlock>
      </div>
    </section>

    <!-- 2. LA BATALLA ÉPICA (Nano vs Vim) -->
    <div class="section-group self-stretch">
      <SectionTitle>1. La Guerra de los Editores</SectionTitle>
      <div class="row q-col-gutter-xl q-mt-sm items-stretch">
        <!-- CARD NANO -->
        <div class="col-12 col-md-6">
          <div class="tool-card nano">
            <div class="tool-header text-center column flex-center">
              <div class="text-h2 q-mb-sm">🛡️</div>
              <h2 class="q-mb-xs">GNU nano</h2>
              <div class="text-green-4 text-subtitle2 text-italic">"Simple. Humano. Efectivo."</div>
            </div>

            <ul class="tool-list q-mt-lg">
              <li>👶 <strong>Curva de aprendizaje:</strong> 5 segundos.</li>
              <li>👀 <strong>Interfaz:</strong> Te muestra los comandos abajo.</li>
              <li>
                ✅ <strong>Ideal para:</strong> Configurar redes, editar <code>.bashrc</code>,
                arreglos rápidos.
              </li>
            </ul>

            <div class="text-center q-mt-auto q-pt-md">
              <q-chip color="green-9" text-color="white" icon="star">Nuestra Elección</q-chip>
            </div>
          </div>
        </div>

        <!-- CARD VIM -->
        <div class="col-12 col-md-6">
          <div class="tool-card vim">
            <div class="tool-header text-center column flex-center">
              <div class="text-h2 q-mb-sm">💀</div>
              <h2 class="q-mb-xs">Vim / Vi</h2>
              <div class="text-red-4 text-subtitle2 text-italic">
                "Poderoso... si logras salir."
              </div>
            </div>

            <ul class="tool-list q-mt-lg">
              <li>📈 <strong>Curva:</strong> Vertical (tipo pared de escalada).</li>
              <li>🧠 <strong>Modos:</strong> Confuso al inicio ("Insert" vs "Command").</li>
              <li>
                ⚠️ <strong>Leyenda:</strong> Programadores atrapados desde 1995 porque no saben
                cerrar (<code>:q!</code>).
              </li>
            </ul>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. DOMINANDO NANO -->
    <div class="section-group self-stretch">
      <SectionTitle>2. Dominando Nano en 1 Minuto</SectionTitle>
      <TextBlock>
        Nano es amigable porque
        <strong>siempre te muestra la "chuleta" (cheat sheet)</strong> abajo. El símbolo
        <code>^</code> significa <strong>Control (Ctrl)</strong>.
      </TextBlock>

      <!-- SIMULADOR VISUAL DE NANO -->
      <div class="nano-terminal q-my-lg font-mono text-white q-pa-md rounded-borders shadow-2">
        <div class="row items-center q-mb-sm text-grey-5 border-b q-pb-xs">
          <div class="col text-center">GNU nano 6.2</div>
          <div class="col text-center">Archivo: robot_config.yaml</div>
        </div>

        <div class="editor-content q-my-md text-green-3">
          ros_domain_id: 30<br />
          node_name: "brazo_robotico"<br />
          max_speed: 1.5 <span class="cursor-blink">_</span>
        </div>

        <div class="nano-commands row q-mt-md q-pt-sm border-t text-caption">
          <div class="col-6 col-md-3">
            <span class="text-black bg-white q-px-xs">^G</span> Ayuda
          </div>
          <div class="col-6 col-md-3">
            <span class="text-black bg-white q-px-xs">^O</span> Guardar
          </div>
          <div class="col-6 col-md-3">
            <span class="text-black bg-white q-px-xs">^X</span> Salir
          </div>
          <div class="col-6 col-md-3">
            <span class="text-black bg-white q-px-xs">^K</span> Cortar
          </div>
        </div>
      </div>

      <AlertBlock type="info" title="💡 El Flujo de Trabajo">
        1. <code>nano archivo.txt</code> (Abres)<br />
        2. Escribes tus cambios.<br />
        3. <strong>Ctrl + O</strong> (Guardar) → <strong>Enter</strong> (Confirmar).<br />
        4. <strong>Ctrl + X</strong> (Salir).
      </AlertBlock>
    </div>

    <!-- 4. VS CODE VS TERMINAL -->
    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>3. ¿Y VS Code? (El elefante en la sala)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            <strong>VS Code</strong> es maravilloso y será tu herramienta principal (95% del
            tiempo). Tiene colores, plugins de Python y GitHub Copilot. <br /><br />
            Pero VS Code <strong>NO funciona en una terminal SSH pura</strong> cuando te conectas a
            un dron volando a 50 metros. <br /><br />
            Ahí es donde <strong>Nano</strong> brilla. Es tu navaja suiza de campo. VS Code es tu
            taller completo.
          </TextBlock>
        </template>

        <template #right>
          <!-- Diagrama de Arquitectura -->
          <div class="tool-card no-border-top text-center q-pa-lg">
            <div class="column items-center">
              <q-icon name="laptop_mac" size="4rem" color="blue-4" class="q-mb-sm" />
              <div class="text-weight-bold text-white">Tu PC de Desarrollo</div>
              <div class="text-caption text-grey-5">Aquí usas VS Code</div>
            </div>

            <div class="q-my-md row justify-center items-center text-grey-6">
              <q-icon name="arrow_downward" size="md" />
              <div class="q-mx-sm border-dashed q-px-sm text-caption">SSH (Solo Texto)</div>
            </div>

            <div class="column items-center">
              <q-icon name="memory" size="4rem" color="green-4" class="q-mb-sm" />
              <div class="text-weight-bold text-white">Cerebro del Robot (RPi/Jetson)</div>
              <div class="text-caption text-green-4 text-weight-bold">¡AQUÍ USAS NANO!</div>
            </div>
          </div>
        </template>
      </SplitBlock>
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
    radial-gradient(circle at center, rgba(34, 197, 94, 0.15), transparent 60%),
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

/* TOOL CARDS (Adaptadas para Editores) */
.tool-card {
  height: 100%;
  padding: 24px;
  border-radius: 16px;
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(255, 255, 255, 0.05);
  display: flex;
  flex-direction: column;
}
.tool-card.nano {
  border-top: 4px solid #4ade80;
} /* Green */
.tool-card.vim {
  border-top: 4px solid #f87171;
} /* Red */
.tool-card.no-border-top {
  border-top: none;
  border: 1px solid rgba(148, 163, 184, 0.2);
}

.tool-list {
  list-style: none;
  padding: 0;
  color: #cbd5e1;
}
.tool-list li {
  margin-bottom: 12px;
}

/* TERMINAL SIMULADA NANO */
.nano-terminal {
  background-color: #1e1e1e;
  border: 1px solid #444;
}
.border-b {
  border-bottom: 1px solid #444;
}
.border-t {
  border-top: 1px solid #444;
}
.cursor-blink {
  background-color: #4caf50;
  color: #1e1e1e;
  animation: blink 1s step-end infinite;
}
@keyframes blink {
  50% {
    opacity: 0;
  }
}

.border-dashed {
  border: 1px dashed rgba(255, 255, 255, 0.2);
  border-radius: 4px;
}

.font-mono {
  font-family: 'Fira Code', monospace;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
