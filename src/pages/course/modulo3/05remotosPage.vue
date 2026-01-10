<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-pink-4 text-weight-bold q-mb-sm">
          MÓDULO 3.5: SINCRONIZACIÓN GLOBAL
        </div>

        <h1 class="hero-title">Repositorios <span class="text-white">Remotos</span></h1>

        <TextBlock>
          Tu código en tu laptop está seguro, pero está aislado. Para trabajar en equipo (o
          simplemente para no perder tu tesis si te roban la laptop), necesitas un espejo en la
          nube. Aprende a conectar tu repositorio local con <strong>GitHub</strong> usando los
          comandos sagrados: <strong>Remote, Push y Pull</strong>.
        </TextBlock>
      </div>
    </section>

    <!-- 2. EL CONCEPTO: ORIGIN -->
    <div class="section-group self-stretch">
      <SectionTitle>1. ¿Qué es "Origin"?</SectionTitle>

      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-5">
          <TextBlock>
            Cuando conectas tu carpeta local con GitHub, Git necesita un apodo para esa dirección
            web larga (https://github.com/...).
            <br /><br />
            Por convención universal, llamamos a ese servidor principal <strong>origin</strong>.
            <br /><br />
            Así que cuando veas <code>origin main</code>, lee: "La rama Main que vive en GitHub".
          </TextBlock>
        </div>

        <div class="col-12 col-md-7">
          <!-- VISUAL CONNECTION -->
          <div
            class="tool-card connection-card relative-position q-pa-xl bg-slate-900 shadow-2 overflow-hidden"
          >
            <div class="row justify-between items-center relative-position z-top">
              <!-- LOCAL -->
              <div class="column items-center z-top">
                <div class="bg-slate-800 q-pa-md rounded-borders shadow-1 border-light">
                  <q-icon name="laptop_mac" size="3.5rem" color="grey-4" />
                </div>
                <div class="text-subtitle2 text-white q-mt-sm text-weight-bold">Tu Laptop</div>
                <div class="text-caption text-grey-6 font-mono text-xs">Local</div>
              </div>

              <!-- CABLE -->
              <div class="col text-center relative-position self-center q-px-md">
                <div class="connection-line"></div>
                <div
                  class="link-badge bg-black q-px-md q-py-sm rounded-borders border-pink shadow-glow-pink"
                >
                  <span class="text-pink-4 text-weight-bold text-caption">origin</span>
                </div>
                <div
                  class="text-caption text-grey-6 q-mt-sm font-mono text-xs text-break opacity-70"
                >
                  https://github.com/...
                </div>
              </div>

              <!-- REMOTE -->
              <div class="column items-center z-top">
                <div class="bg-slate-800 q-pa-md rounded-borders shadow-1 border-light">
                  <q-icon name="cloud_circle" size="3.5rem" color="white" />
                </div>
                <div class="text-subtitle2 text-white q-mt-sm text-weight-bold">GitHub</div>
                <div class="text-caption text-grey-6 font-mono text-xs">Remote</div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. PUSH VS PULL (EL FLUJO DE SANGRE) -->
    <div class="section-group self-stretch">
      <SectionTitle>2. El Latido del Código: Push y Pull</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            La sincronización no es automática (como Dropbox). Tú decides cuándo ocurre.
          </TextBlock>
          <ul class="q-pl-md q-mt-md text-grey-4 tool-list">
            <li>
              🚀 <strong>Push (Empujar):</strong> Subir tus cambios locales a la nube. "¡Miren lo
              que hice!"
            </li>
            <li>
              ⬇️ <strong>Pull (Jalar):</strong> Descargar los cambios de la nube a tu local.
              "¡Actualízame!"
            </li>
          </ul>
          <div class="q-mt-lg">
            <AlertBlock type="info" title="Regla de Oro">
              Siempre haz <code>pull</code> antes de hacer <code>push</code>. Si no, chocarás con el
              trabajo de otros.
            </AlertBlock>
          </div>
        </template>

        <template #right>
          <div
            class="tool-card flow-vertical q-pa-lg bg-slate-900 border-light flex column justify-center"
          >
            <!-- PUSH ACTION -->
            <div
              class="row items-center justify-between q-mb-lg group-hover q-pa-md rounded-borders bg-dark-soft transition-hover"
            >
              <div
                class="bg-black q-pa-sm rounded-borders font-mono text-pink-4 text-xs border-pink-dim"
              >
                git push origin main
              </div>
              <div class="row items-center">
                <span class="text-grey-5 q-mr-sm text-caption text-uppercase">Subir</span>
                <q-icon name="arrow_upward" color="pink-4" size="1.8rem" class="bounce-up" />
              </div>
            </div>

            <div class="separator-line bg-grey-8 full-width q-my-sm" style="height: 1px"></div>

            <!-- PULL ACTION -->
            <div
              class="row items-center justify-between q-mt-lg group-hover q-pa-md rounded-borders bg-dark-soft transition-hover"
            >
              <div
                class="bg-black q-pa-sm rounded-borders font-mono text-cyan-4 text-xs border-cyan-dim"
              >
                git pull origin main
              </div>
              <div class="row items-center">
                <span class="text-grey-5 q-mr-sm text-caption text-uppercase">Bajar</span>
                <q-icon name="arrow_downward" color="cyan-4" size="1.8rem" class="bounce-down" />
              </div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 4. AUTENTICACIÓN (EL TOKEN DE ACCESO) -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>3. La Llave de Seguridad (SSH/Token)</SectionTitle>
      <AlertBlock type="warning" title="Adiós a las Contraseñas">
        Desde 2021, GitHub <strong>ya no acepta tu contraseña</strong> de usuario para hacer push.
        Es por seguridad. Debes configurar una <strong>Llave SSH</strong> (recomendado) o usar un
        <strong>Personal Access Token (PAT)</strong>.
      </AlertBlock>

      <div class="row q-mt-lg q-col-gutter-lg">
        <!-- SSH CARD -->
        <div class="col-12 col-md-6">
          <div class="tool-card auth-card ssh-style bg-slate-800 shadow-2 h-full column">
            <div class="row items-center q-mb-md border-bottom-dark q-pb-sm">
              <q-icon name="vpn_key" color="green-4" size="1.8rem" />
              <div class="text-subtitle1 text-white q-ml-sm text-weight-bold">SSH Key (Pro)</div>
            </div>
            <p class="text-grey-4 text-caption col-grow">
              Generas un archivo en tu PC y subes la parte pública a GitHub. Nunca más te pide
              contraseña.
            </p>
            <div
              class="bg-black q-pa-md rounded-borders font-mono text-xs text-green-4 q-mt-auto border-green-dim text-center"
            >
              ssh-keygen -t ed25519 -C "email"
            </div>
          </div>
        </div>

        <!-- PAT CARD -->
        <div class="col-12 col-md-6">
          <div class="tool-card auth-card pat-style bg-slate-800 shadow-2 h-full column">
            <div class="row items-center q-mb-md border-bottom-dark q-pb-sm">
              <q-icon name="password" color="orange-4" size="1.8rem" />
              <div class="text-subtitle1 text-white q-ml-sm text-weight-bold">Token (PAT)</div>
            </div>
            <p class="text-grey-4 text-caption col-grow">
              Generas un código secreto en la web de GitHub y lo usas como password. Caduca cada
              cierto tiempo.
            </p>
            <div
              class="bg-black q-pa-md rounded-borders font-mono text-xs text-orange-4 q-mt-auto border-orange-dim text-center"
            >
              ghp_A1b2C3d4...
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 5. ERROR CLÁSICO: REJECTED -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>4. Pesadilla: "Updates were rejected"</SectionTitle>
      <div class="tool-card error-card bg-slate-900 q-pa-none border-red shadow-2 overflow-hidden">
        <!-- TERMINAL ERROR -->
        <div class="bg-black q-pa-md border-bottom-red-dim">
          <div class="text-red-4 font-mono text-xs">
            ! [rejected] main -> main (fetch first)<br />
            error: failed to push some refs to 'github.com/...'
          </div>
        </div>

        <div class="row items-center q-col-gutter-lg q-pa-lg">
          <div class="col-12 col-md-7">
            <div class="text-subtitle2 text-white q-mb-xs">¿Por qué pasa esto?</div>
            <p class="text-caption text-grey-4 q-mb-none">
              Alguien (quizás tú desde otra PC) subió cambios a GitHub que tú no tienes en tu
              laptop. GitHub protege el repositorio y te dice:
              <em
                >"No puedes sobrescribir esto. Primero descarga lo nuevo, mézclalo con lo tuyo, y
                luego sube todo junto".</em
              >
            </p>
          </div>
          <div class="col-12 col-md-5 text-center">
            <div class="bg-green-9 text-white q-pa-md rounded-borders shadow-1">
              <div class="text-weight-bold text-caption text-uppercase q-mb-sm">La Solución</div>
              <div class="font-mono text-xs bg-black q-pa-xs rounded text-green-2 border-green-dim">
                git pull origin main
              </div>
              <div class="text-caption q-my-xs text-grey-3 font-italic text-xs">
                ...arregla conflictos...
              </div>
              <div class="font-mono text-xs bg-black q-pa-xs rounded text-green-2 border-green-dim">
                git push origin main
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 6. CLONAR (EMPEZAR DE CERO) -->
    <div class="section-group self-stretch q-mt-xl q-mb-xl">
      <div class="bg-slate-800 q-pa-lg rounded-borders border-accent text-center shadow-glow-pink">
        <q-icon name="content_copy" size="md" color="white" class="q-mb-sm" />
        <div class="text-h6 text-white">Descargar el Trabajo de Otros</div>
        <p class="text-grey-4 q-mb-md">
          ¿Viste un robot genial en GitHub y quieres probarlo? No descargues el ZIP. Usa
          <code>clone</code> para bajar una copia viva, con todo su historial.
        </p>
        <div
          class="bg-black q-pa-md rounded-borders font-mono text-body2 text-pink-4 inline-block text-break border-light"
        >
          git clone https://github.com/ros-planning/navigation2.git
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
    radial-gradient(circle at center, rgba(236, 72, 153, 0.15), transparent 60%),
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

/* CONNECTION CARD */
.connection-card {
  border-top: 4px solid #ec4899;
}
.z-top {
  z-index: 2;
  position: relative;
}

.connection-line {
  position: absolute;
  top: 50%;
  left: 0;
  width: 100%;
  height: 2px;
  background: linear-gradient(90deg, transparent 0%, #ec4899 50%, transparent 100%);
  opacity: 0.5;
  z-index: 1;
}
.link-badge {
  position: relative;
  z-index: 2;
  display: inline-block;
  background: #000;
}
.border-pink {
  border: 1px solid #ec4899;
}
.shadow-glow-pink {
  box-shadow: 0 0 20px rgba(236, 72, 153, 0.4);
}

/* FLOW VERTICAL */
.bounce-up {
  animation: bounceUp 2s infinite ease-in-out;
}
.bounce-down {
  animation: bounceDown 2s infinite ease-in-out;
}

@keyframes bounceUp {
  0%,
  100% {
    transform: translateY(0);
    opacity: 0.5;
  }
  50% {
    transform: translateY(-8px);
    opacity: 1;
  }
}
@keyframes bounceDown {
  0%,
  100% {
    transform: translateY(0);
    opacity: 0.5;
  }
  50% {
    transform: translateY(8px);
    opacity: 1;
  }
}
.transition-hover {
  transition: background 0.3s;
}
.transition-hover:hover {
  background: rgba(255, 255, 255, 0.05);
}
.border-pink-dim {
  border: 1px solid rgba(236, 72, 153, 0.3);
}
.border-cyan-dim {
  border: 1px solid rgba(34, 211, 238, 0.3);
}

/* AUTH CARDS */
.auth-card {
  padding: 24px;
  transition: transform 0.3s;
}
.auth-card:hover {
  transform: translateY(-5px);
}
.ssh-style {
  border-top: 4px solid #4ade80;
}
.pat-style {
  border-top: 4px solid #fb923c;
}
.h-full {
  height: 100%;
}
.col-grow {
  flex-grow: 1;
}
.border-green-dim {
  border: 1px solid rgba(74, 222, 128, 0.2);
}
.border-orange-dim {
  border: 1px solid rgba(251, 146, 60, 0.2);
}

/* ERROR CARD */
.border-red {
  border-left: 4px solid #ef4444;
}
.border-bottom-red-dim {
  border-bottom: 1px solid rgba(239, 68, 68, 0.2);
}

/* UTILS */
.font-mono {
  font-family: 'Fira Code', monospace;
}
.text-xs {
  font-size: 0.8rem;
}
.text-break {
  word-break: break-all;
}
.bg-slate-800 {
  background: #1e293b;
}
.bg-slate-900 {
  background: #0f172a;
}
.bg-dark-soft {
  background: rgba(0, 0, 0, 0.2);
}
.border-light {
  border: 1px solid rgba(255, 255, 255, 0.1);
}
.border-bottom-dark {
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}
.opacity-70 {
  opacity: 0.7;
}
.tool-list {
  list-style: none;
  padding: 0;
}
.tool-list li {
  margin-bottom: 10px;
  font-size: 1rem;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
  .connection-line {
    display: none;
  }
  .connection-card .row {
    flex-direction: column;
    gap: 24px;
  }
}
</style>
