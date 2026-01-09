<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO: EL GRAN CONCEPTO (Estilo Enciclopedia Técnica) -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-accent text-weight-bold q-mb-sm">MÓDULO 0: PREPARACIÓN</div>
        <h1 class="hero-title">Navegación en <span class="text-primary">Terminal</span></h1>

        <TextBlock>
          Como ingeniero en robótica, el 90% de tu tiempo estarás conectado a robots o servidores
          que
          <strong>no tienen monitor ni mouse</strong>. Solo tendrás una pantalla negra y tu teclado.
          Dominar la terminal es la diferencia entre un usuario y un desarrollador profesional.
        </TextBlock>
      </div>
    </section>

    <!-- 2. CONCEPTOS CLAVE (Estilo Tarjetas Mini) -->
    <div class="section-group self-stretch">
      <SectionTitle>Conceptos Fundamentales</SectionTitle>
      <div class="glossary-grid-mini">
        <div class="term-card">
          <div class="term-title text-green-4">Shell (Bash)</div>
          <div class="term-desc">
            El intérprete de comandos. Es el programa que recibe lo que escribes y se lo ordena al
            sistema operativo.
          </div>
        </div>
        <div class="term-card">
          <div class="term-title text-purple-4">Filesystem</div>
          <div class="term-desc">
            En Linux no hay "Unidad C:". Todo nace de la raíz (/). Es un árbol invertido de
            directorios.
          </div>
        </div>
        <div class="term-card">
          <div class="term-title text-blue-4">Path (Ruta)</div>
          <div class="term-desc">
            La dirección exacta de un archivo. Puede ser Absoluta (desde /) o Relativa (desde donde
            estás).
          </div>
        </div>
        <div class="term-card">
          <div class="term-title text-orange-4">~ (Tilde)</div>
          <div class="term-desc">
            Atajo universal para tu carpeta personal (/home/tu_usuario). Tu "casa" en el sistema.
          </div>
        </div>
      </div>
    </div>

    <!-- 3. COMANDO PWD -->
    <div class="section-group self-stretch">
      <SectionTitle>1. Ubicación Actual (pwd)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            El comando <code>pwd</code> (<em>Print Working Directory</em>) es tu GPS. Te devuelve la
            ruta absoluta desde la raíz hasta donde estás parado. <br /><br />
            Siempre que te sientas perdido, ejecútalo para saber exactamente en qué rama del árbol
            estás.
          </TextBlock>
        </template>
        <template #right>
          <div class="terminal-window q-pa-md rounded-borders shadow-2">
            <div class="terminal-header row items-center q-mb-sm">
              <div class="window-btn red q-mr-xs"></div>
              <div class="window-btn yellow q-mr-xs"></div>
              <div class="window-btn green"></div>
              <div class="col text-center text-grey-6 text-caption">bash</div>
            </div>
            <CodeBlock
              :hide-header="true"
              lang="bash"
              content="alexander@robot:~$ pwd
/home/alexander"
            />
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 4. COMANDO LS -->
    <div class="section-group self-stretch">
      <SectionTitle>2. Exploración (ls)</SectionTitle>
      <TextBlock>
        <code>ls</code> lista el contenido actual. Pero en ingeniería, los detalles importan.
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <div class="col-12 col-md-6">
          <div class="def-card">
            <div class="def-header">
              <q-icon name="list" color="primary" size="sm" />
              <div class="def-title">ls -l (Long)</div>
            </div>
            <div class="def-body">
              Muestra formato de lista detallada. Vital para ver <strong>permisos</strong> (rwx),
              dueño y tamaño de archivos.
            </div>
            <div class="def-meta">Uso: Verificar permisos de scripts</div>
          </div>
        </div>
        <div class="col-12 col-md-6">
          <div class="def-card">
            <div class="def-header">
              <q-icon name="visibility" color="secondary" size="sm" />
              <div class="def-title">ls -a (All)</div>
            </div>
            <div class="def-body">
              Muestra archivos ocultos (los que inician con <code>.</code>). Aquí viven las
              configuraciones como <code>.bashrc</code>.
            </div>
            <div class="def-meta">Uso: Configurar entorno ROS</div>
          </div>
        </div>
      </div>
    </div>

    <!-- 5. WORKFLOW Y TRUCOS -->
    <div class="section-group self-stretch">
      <SectionTitle>3. Movimiento Rápido (cd)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            La terminal tiene atajos de "teletransportación" que te ahorrarán horas de escritura.
          </TextBlock>
          <div class="glossary-list">
            <div class="glossary-item">
              <strong>cd ..</strong> — Subir un nivel (ir a la carpeta padre).
            </div>
            <div class="glossary-item"><strong>cd ~</strong> — Ir directamente a Home.</div>
            <div class="glossary-item">
              <strong>cd -</strong> — Volver a la carpeta anterior (como el botón "atrás" del
              navegador).
            </div>
          </div>
        </template>
        <template #right>
          <AlertBlock type="success" title="Tip de Productividad">
            Usa la tecla <strong>TAB</strong> para autocompletar nombres. <br />
            Escribe <code>cd rob</code> + TAB → se convierte en <code>cd robot_ws/</code>. ¡Evita
            errores de dedo!
          </AlertBlock>
        </template>
      </SplitBlock>
    </div>

    <!-- RETO FINAL -->
    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>🏆 Reto de Ingeniería</SectionTitle>
      <TextBlock>
        Predice la salida final antes de ejecutar este bloque. ¿En qué carpeta terminarás?
      </TextBlock>

      <CodeBlock
        title="Simulación de Navegación"
        lang="bash"
        content="cd ~
mkdir -p proyecto_xyz/logs
cd proyecto_xyz/logs
touch error.log
cd ../..
pwd"
        :copyable="true"
      />
    </div>
  </q-page>
</template>

<script setup lang="ts">
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
</script>

<style scoped>
/* ESTILOS GLOBALES DE LA PÁGINA (Idénticos al modelo 'Glosario') */
.intro-hero,
.section-group {
  width: 100%;
  max-width: 1000px; /* Ancho controlado para lectura óptima */
  margin: 0 auto 3.5rem auto;
}

/* HERO SECTION PREMIUM */
.intro-hero {
  padding: 4rem 2rem;
  background:
    radial-gradient(circle at center, rgba(74, 222, 128, 0.1), transparent 60%),
    /* Verde sutil para Modulo 0 */ rgba(15, 23, 42, 0.8);
  backdrop-filter: blur(20px);
  border-radius: 24px;
  border: 1px solid rgba(148, 163, 184, 0.2);
  text-align: center;
  margin-bottom: 4rem;
}

.hero-title {
  font-size: 3.5rem;
  font-weight: 800;
  margin: 0 0 1.5rem 0;
  line-height: 1.1;
  color: #f8fafc;
}

/* CARDS PEQUEÑAS (GLOSARIO MINI) */
.glossary-grid-mini {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 16px;
  margin-top: 24px;
}

.term-card {
  background: rgba(30, 41, 59, 0.5);
  padding: 16px;
  border-radius: 12px;
  border: 1px solid rgba(148, 163, 184, 0.1);
  transition: transform 0.2s;
}
.term-card:hover {
  transform: translateY(-3px);
  background: rgba(30, 41, 59, 0.8);
}

.term-title {
  font-weight: 700;
  font-size: 1rem;
  margin-bottom: 6px;
}

.term-desc {
  font-size: 0.85rem;
  color: #94a3b8;
  line-height: 1.4;
}

/* CARDS DEFINICIÓN (LS COMMANDS) */
.def-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.15);
  border-radius: 16px;
  padding: 24px;
  height: 100%;
}

.def-header {
  display: flex;
  align-items: center;
  gap: 10px;
  margin-bottom: 12px;
}

.def-title {
  margin: 0;
  font-size: 1.1rem;
  color: #f1f5f9;
  font-weight: 600;
}

.def-body {
  color: #cbd5e1;
  font-size: 0.95rem;
  line-height: 1.5;
  margin-bottom: 16px;
}

.def-meta {
  font-size: 0.8rem;
  color: #64748b;
  font-family: 'Fira Code', monospace;
  background: rgba(0, 0, 0, 0.2);
  padding: 6px;
  border-radius: 6px;
  display: inline-block;
}

/* LISTA GLOSARIO (SHORTCUTS) */
.glossary-list {
  display: flex;
  flex-direction: column;
  gap: 12px;
  margin-top: 16px;
}

.glossary-item {
  padding: 12px;
  background: rgba(255, 255, 255, 0.03);
  border-left: 3px solid #38bdf8;
  border-radius: 0 8px 8px 0;
  color: #e2e8f0;
  font-size: 0.95rem;
}

/* TERMINAL WINDOW */
.terminal-window {
  background: #1e1e1e;
  border: 1px solid #333;
}
.window-btn {
  width: 10px;
  height: 10px;
  border-radius: 50%;
}
.window-btn.red {
  background: #ff5f56;
}
.window-btn.yellow {
  background: #ffbd2e;
}
.window-btn.green {
  background: #27c93f;
}

/* RESPONSIVE */
@media (max-width: 768px) {
  .hero-title {
    font-size: 2.5rem;
  }
  .intro-hero {
    padding: 3rem 1.5rem;
  }
}
</style>
