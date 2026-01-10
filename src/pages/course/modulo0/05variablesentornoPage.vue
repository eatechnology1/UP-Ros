<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-accent text-weight-bold q-mb-sm">
          MÓDULO 0: CONFIGURACIÓN INVISIBLE
        </div>

        <h1 class="hero-title">Variables de <span class="text-primary">Entorno</span></h1>

        <TextBlock>
          ¿Por qué tu terminal sabe dónde está Python pero no dónde está ROS 2? Descubre la
          "memoria" de tu sistema y aprende a manipular el ADN de tu sesión para evitar el temido
          "Command not found".
        </TextBlock>
      </div>
    </section>

    <!-- 2. LA METÁFORA DE LA MOCHILA -->
    <div class="section-group self-stretch">
      <SectionTitle>1. La Mochila Invisible</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Cada vez que abres una terminal, Linux crea un "explorador" (tu sesión) que nace con una
            <strong>mochila</strong> llena de notas. <br /><br />
            Esas notas son las <strong>Variables de Entorno</strong>:
          </TextBlock>
          <ul class="tool-list q-pl-md">
            <li>📍 <strong>HOME:</strong> "Tu casa es <code>/home/alexander</code>".</li>
            <li>👤 <strong>USER:</strong> "Te llamas <code>alexander</code>".</li>
            <li>🛣️ <strong>PATH:</strong> "Las herramientas están aquí...".</li>
          </ul>
        </template>

        <template #right>
          <!-- TARJETA VISUAL DE VARIABLE -->
          <div class="tool-card env-var relative-position">
            <div class="absolute-top-right q-pa-md">
              <q-icon name="fingerprint" color="grey-7" size="3rem" />
            </div>

            <div class="text-caption text-grey-5 font-mono q-mb-xs">STRUCTURE</div>
            <div class="text-h4 text-white font-mono q-mb-md">
              <span class="text-accent">CLAVE</span>=<span class="text-green-4">"Valor"</span>
            </div>

            <div class="border-b q-mb-md"></div>

            <div class="text-caption text-grey-4 q-mb-sm">Ejemplo Real:</div>
            <div class="bg-dark q-pa-sm rounded-borders font-mono text-code">
              <span class="text-purple-3">ROS_DISTRO</span>="jazzy"<br />
              <span class="text-purple-3">EDITOR</span>="nano"
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 3. LA REINA PATH -->
    <div class="section-group self-stretch">
      <SectionTitle>2. La Variable Reina: $PATH</SectionTitle>
      <TextBlock>
        Cuando escribes <code>python3</code>, Linux busca <strong>solo</strong> en las carpetas
        listadas en <code>PATH</code>. <br /><br />
        <strong>El problema de ROS 2:</strong> Se instala en <code>/opt/ros/jazzy</code>, una
        carpeta que Linux ignora por defecto.
      </TextBlock>

      <div class="q-mt-md">
        <AlertBlock type="warning" title="🚨 Command 'ros2' not found">
          Si escribes <code>ros2</code> en una terminal nueva, fallará. Tu sistema no sabe que ROS
          existe hasta que le des el mapa.
        </AlertBlock>
      </div>
    </div>

    <!-- 4. EL RITUAL DEL SOURCE -->
    <div class="section-group self-stretch">
      <SectionTitle>3. El Ritual del "Source"</SectionTitle>
      <SplitBlock>
        <template #left>
          <div class="text-h6 text-primary q-mb-sm">La Magia Temporal</div>
          <TextBlock>
            El comando <code>source .../setup.bash</code> inyecta las rutas de ROS en tu mochila
            (PATH) instantáneamente.
          </TextBlock>
          <div class="q-my-md">
            <!-- CORREGIDO: language -> lang, code -> content -->
            <CodeBlock lang="bash" content="source /opt/ros/jazzy/setup.bash" />
          </div>
        </template>

        <template #right>
          <div class="tool-card amnesia">
            <div class="tool-header">
              <q-icon name="history_toggle_off" size="sm" />
              <h4 class="text-h6 text-white q-my-none">Amnesia de Terminal</h4>
            </div>
            <p class="q-my-sm text-grey-4">
              Las terminales tienen memoria de pez. Si cierras la ventana, la mochila se vacía. Las
              variables son <strong>temporales</strong>.
            </p>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 5. .BASHRC -->
    <div class="section-group self-stretch">
      <SectionTitle>4. La Solución Definitiva: .bashrc</SectionTitle>
      <TextBlock>
        El archivo <code>~/.bashrc</code> es un script que se ejecuta
        <strong>automáticamente</strong> al abrir una terminal. Es el lugar para tatuar tus
        variables.
      </TextBlock>

      <div class="q-mt-md">
        <!-- CORREGIDO: language -> lang, code -> content -->
        <CodeBlock
          lang="bash"
          title="Automatizando el entorno"
          content="# 1. Abre tu configuración
nano ~/.bashrc

# 2. Agrega al final:
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=30

# 3. Guarda y recarga
source ~/.bashrc"
        />
      </div>
    </div>

    <!-- 6. ROS_DOMAIN_ID -->
    <div class="section-group self-stretch">
      <SectionTitle>5. Caso Crítico: ROS_DOMAIN_ID</SectionTitle>

      <!-- Card Especial Destacada -->
      <div class="tool-card domain-id q-pa-lg">
        <div class="row items-center q-mb-md">
          <q-icon name="router" size="md" color="white" class="q-mr-md" />
          <h3 class="text-h5 text-white q-my-none">Tu Identidad en la Red</h3>
        </div>

        <p class="text-grey-3 q-mb-lg">
          Si 20 estudiantes usan ROS 2 en la misma Wi-Fi con la config por defecto,
          <strong>¡todos controlarán los robots de todos!</strong>
        </p>

        <div class="row q-col-gutter-md">
          <div class="col-12 col-md-6">
            <div class="bg-dark q-pa-md rounded-borders text-center border-red">
              <div class="text-red-4 text-weight-bold">SIN ID (Default 0)</div>
              <div class="text-caption text-grey-5">Caos total. Broadcast público.</div>
            </div>
          </div>
          <div class="col-12 col-md-6">
            <div class="bg-dark q-pa-md rounded-borders text-center border-green">
              <div class="text-green-4 text-weight-bold font-mono">export ROS_DOMAIN_ID=30</div>
              <div class="text-caption text-grey-5">Canal privado #30.</div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 7. CHEAT SHEET -->
    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>Comandos de Diagnóstico</SectionTitle>
      <div class="row q-gutter-md justify-center">
        <q-chip color="dark" text-color="cyan-3" icon="terminal" class="glossary-chip">
          printenv | grep ROS
        </q-chip>
        <q-chip color="dark" text-color="cyan-3" icon="terminal" class="glossary-chip">
          echo $PATH
        </q-chip>
        <q-chip color="dark" text-color="cyan-3" icon="terminal" class="glossary-chip">
          export VAR="Hola"
        </q-chip>
      </div>
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
    radial-gradient(circle at center, rgba(56, 189, 248, 0.15), transparent 60%),
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
  padding: 24px;
  border-radius: 16px;
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(255, 255, 255, 0.05);
  display: flex;
  flex-direction: column;
}
.tool-card.env-var {
  border-top: 4px solid #a855f7;
} /* Purple */
.tool-card.amnesia {
  border-top: 4px solid #f59e0b;
} /* Orange */
.tool-card.domain-id {
  border-top: 4px solid #38bdf8; /* Cyan */
  background: linear-gradient(145deg, rgba(30, 41, 59, 0.6), rgba(15, 23, 42, 0.9));
}

.tool-header {
  display: flex;
  align-items: center;
  gap: 12px;
}

/* UTILIDADES */
.font-mono {
  font-family: 'Fira Code', monospace;
}
.text-code {
  font-size: 0.9rem;
}
.border-b {
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}

.border-red {
  border: 1px solid rgba(248, 113, 113, 0.3);
}
.border-green {
  border: 1px solid rgba(74, 222, 128, 0.3);
}

.glossary-chip {
  border: 1px solid rgba(255, 255, 255, 0.1);
  padding: 16px 12px;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
