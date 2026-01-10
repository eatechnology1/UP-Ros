<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-accent text-weight-bold q-mb-sm">
          MÓDULO 0: LÍNEA DE COMANDOS
        </div>

        <h1 class="hero-title">Gestión de <span class="text-primary">Archivos</span></h1>

        <TextBlock>
          Ya sabes moverte por el sistema. Ahora aprenderás a manipular la materia: crear archivos,
          duplicarlos, moverlos y destruirlos. En robótica, esto es el día a día para gestionar logs
          y configuraciones.
        </TextBlock>
      </div>
    </section>

    <!-- 2. TOUCH: CREAR -->
    <div class="section-group self-stretch">
      <SectionTitle>1. Crear Archivos Vacíos (touch)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            El comando <code>touch</code> se usa para crear un archivo totalmente vacío. Es como
            sacar una hoja de papel en blanco. <br /><br />
            Si el archivo ya existe, <code>touch</code> solo actualiza su "fecha de modificación"
            sin borrar el contenido.
          </TextBlock>
        </template>
        <template #right>
          <CodeBlock
            title="Crear script vacío"
            lang="bash"
            content="touch nodo_control.py
ls -l"
            :copyable="true"
          />
        </template>
      </SplitBlock>
    </div>

    <!-- 3. CP: COPIAR -->
    <div class="section-group self-stretch">
      <SectionTitle>2. Copiar Archivos (cp)</SectionTitle>
      <TextBlock>
        El comando <code>cp</code> (Copy) necesita dos argumentos: <strong>Origen</strong> y
        <strong>Destino</strong>.
      </TextBlock>

      <div class="q-my-md">
        <AlertBlock type="info" title="Sintaxis Básica">
          <code>cp [archivo_origen] [archivo_destino]</code>
        </AlertBlock>
      </div>

      <CodeBlock
        title="Ejemplo: Backup"
        lang="bash"
        content="# Crear el original
touch mapa.yaml

# Hacer una copia de seguridad
cp mapa.yaml mapa_backup.yaml"
        :copyable="true"
      />

      <div class="q-mt-md">
        <AlertBlock type="warning" title="Copiar Carpetas">
          Para copiar una carpeta entera y su contenido, es obligatorio usar la bandera
          <code>-r</code> (recursivo).
          <br />
          Ejemplo: <code>cp -r carpeta_original carpeta_copia</code>
        </AlertBlock>
      </div>
    </div>

    <!-- 4. MV: MOVER Y RENOMBRAR -->
    <div class="section-group self-stretch">
      <SectionTitle>3. Mover y Renombrar (mv)</SectionTitle>
      <TextBlock>
        En Linux, <strong>Mover</strong> y <strong>Renombrar</strong> son la misma operación:
        <code>mv</code> (Move).
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <div class="col-12 col-md-6">
          <!-- Convertido a Tool Card con borde secundario -->
          <div class="tool-card mv-move">
            <div class="tool-header">
              <q-icon name="drive_file_move" size="sm" />
              <h4 class="text-h6 text-white q-my-none">Caso A: Mover</h4>
            </div>
            <p class="q-my-sm text-grey-4">
              Si el destino es una <strong>carpeta</strong> existente, el archivo se mete dentro de
              ella.
            </p>
            <CodeBlock lang="bash" content="mv archivo.txt carpeta_destino/" :copyable="false" />
          </div>
        </div>

        <div class="col-12 col-md-6">
          <!-- Convertido a Tool Card con borde primario -->
          <div class="tool-card mv-rename">
            <div class="tool-header">
              <q-icon name="edit" size="sm" />
              <h4 class="text-h6 text-white q-my-none">Caso B: Renombrar</h4>
            </div>
            <p class="q-my-sm text-grey-4">
              Si el destino es un <strong>nombre nuevo</strong>, el archivo cambia de nombre en el
              mismo lugar.
            </p>
            <CodeBlock lang="bash" content="mv viejo.txt nuevo.txt" :copyable="false" />
          </div>
        </div>
      </div>
    </div>

    <!-- 5. RM: ELIMINAR -->
    <div class="section-group self-stretch">
      <SectionTitle>4. Eliminar (rm)</SectionTitle>

      <AlertBlock type="danger" title="⚠️ ADVERTENCIA CRÍTICA">
        En la terminal <strong>NO EXISTE LA PAPELERA DE RECICLAJE</strong>.
        <br />
        Cuando borras algo con <code>rm</code>, desaparece para siempre. No hay "Ctrl+Z".
      </AlertBlock>

      <TextBlock class="q-mt-md">
        Para borrar archivos usamos <code>rm</code>. Para carpetas, necesitamos fuerza bruta
        (recursividad).
      </TextBlock>

      <SplitBlock class="q-mt-md">
        <template #left>
          <div class="text-subtitle2 q-mb-xs text-grey-4">Borrar archivo simple</div>
          <CodeBlock lang="bash" content="rm archivo_feo.txt" />
        </template>
        <template #right>
          <div class="text-subtitle2 q-mb-xs text-grey-4">Borrar carpeta (Recursivo + Force)</div>
          <CodeBlock lang="bash" content="rm -rf carpeta_fea/" />
        </template>
      </SplitBlock>
    </div>

    <!-- 6. RETO -->
    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>🏆 Reto de Limpieza</SectionTitle>
      <TextBlock>
        Practica creando un desastre controlado y luego limpiándolo. Ejecuta línea por línea:
      </TextBlock>

      <div class="q-mt-md">
        <StepsBlock
          :steps="[
            'Crear carpeta temp: <code>mkdir temp</code>',
            'Entrar en ella: <code>cd temp</code>',
            'Crear 3 archivos: <code>touch a.txt b.txt c.txt</code>',
            'Renombrar a.txt: <code>mv a.txt final.txt</code>',
            'Borrar b y c: <code>rm b.txt c.txt</code>',
            'Salir: <code>cd ..</code>',
            'Borrar todo: <code>rm -rf temp</code>',
          ]"
        />
      </div>
    </div>
  </q-page>
</template>

<script setup lang="ts">
import SectionTitle from 'components/content/SectionTitle.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
import StepsBlock from 'components/content/StepsBlock.vue';
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
    radial-gradient(circle at center, rgba(16, 185, 129, 0.15), transparent 60%),
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

/* TOOL CARDS (Reutilizadas para MV) */
.tool-card {
  height: 100%;
  padding: 24px;
  border-radius: 16px;
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(255, 255, 255, 0.05);
  display: flex;
  flex-direction: column;
}
.tool-card.mv-move {
  border-top: 4px solid #26a69a;
} /* Secondary Teal/Greenish */
.tool-card.mv-rename {
  border-top: 4px solid #1976d2;
} /* Primary Blue */

.tool-header {
  display: flex;
  align-items: center;
  gap: 12px;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
