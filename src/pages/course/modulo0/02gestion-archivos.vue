<template>
  <q-page class="q-pa-lg column items-center">
    <!-- ========================================================================================
         1. HERO SECTION
    ======================================================================================== -->
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

    <!-- ========================================================================================
         2. CONTENIDO PRINCIPAL
    ======================================================================================== -->

    <!-- 1. TOUCH: CREAR ARCHIVOS -->
    <div class="section-group self-stretch">
      <SectionTitle>1. Crear Archivos Vacíos (touch)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            El comando <code>touch</code> se usa para crear un archivo totalmente vacío. Es como
            sacar una hoja de papel en blanco. <br /><br />
            Si el archivo ya existe, <code>touch</code> actualiza su "fecha de modificación" sin
            borrar el contenido.
          </TextBlock>
        </template>
        <template #right>
          <CodeBlock
            title="Crear un script Python vacío"
            lang="bash"
            content="touch nodo_control.py
ls -l"
            :copyable="true"
          />
        </template>
      </SplitBlock>
    </div>

    <!-- 2. CP: COPIAR -->
    <div class="section-group self-stretch">
      <SectionTitle>2. Copiar Archivos (cp)</SectionTitle>
      <TextBlock>
        El comando <code>cp</code> (Copy) necesita dos cosas: <strong>¿Qué copio?</strong> (Origen)
        y <strong>¿A dónde lo copio?</strong> (Destino).
      </TextBlock>

      <div class="q-my-md">
        <AlertBlock type="info" title="Sintaxis">
          <code>cp [origen] [destino]</code>
        </AlertBlock>
      </div>

      <CodeBlock
        title="Hacer un backup"
        lang="bash"
        content="# Crear el original
touch mapa.yaml

# Hacer una copia
cp mapa.yaml mapa_backup.yaml"
        :copyable="true"
      />

      <div class="q-mt-md">
        <AlertBlock type="warning" title="Copiar Carpetas">
          Para copiar una carpeta entera y su contenido, necesitas la bandera
          <code>-r</code> (recursivo).
          <br />
          Ejemplo: <code>cp -r carpeta_original carpeta_copia</code>
        </AlertBlock>
      </div>
    </div>

    <!-- 3. MV: MOVER Y RENOMBRAR -->
    <div class="section-group self-stretch">
      <SectionTitle>3. Mover y Renombrar (mv)</SectionTitle>
      <TextBlock>
        Aquí viene algo que confunde a los principiantes. En Linux, <strong>Mover</strong> y
        <strong>Renombrar</strong> son el mismo comando: <code>mv</code> (Move).
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <div class="col-12 col-md-6">
          <div class="concept-card">
            <div class="text-h6 text-secondary q-mb-sm">Caso A: Mover</div>
            <p>Si el destino es una <strong>carpeta</strong>, el archivo se mete dentro.</p>
            <CodeBlock lang="bash" content="mv archivo.txt carpeta_destino/" :copyable="false" />
          </div>
        </div>
        <div class="col-12 col-md-6">
          <div class="concept-card">
            <div class="text-h6 text-primary q-mb-sm">Caso B: Renombrar</div>
            <p>Si el destino es un <strong>nombre nuevo</strong>, el archivo cambia de nombre.</p>
            <CodeBlock
              lang="bash"
              content="mv nombre_viejo.txt nombre_nuevo.txt"
              :copyable="false"
            />
          </div>
        </div>
      </div>
    </div>

    <!-- 4. RM: EL PELIGRO DE BORRAR -->
    <div class="section-group self-stretch">
      <SectionTitle>4. Eliminar (rm)</SectionTitle>

      <AlertBlock type="danger" title="⚠️ ADVERTENCIA DE SEGURIDAD">
        En la terminal de Linux <strong>NO EXISTE LA PAPELERA DE RECICLAJE</strong>.
        <br />
        Cuando borras algo con <code>rm</code>, desaparece para siempre. No hay "Deshacer".
      </AlertBlock>

      <TextBlock class="q-mt-md">
        Para borrar archivos usamos <code>rm</code> (Remove). Para carpetas, necesitamos fuerza
        bruta.
      </TextBlock>

      <SplitBlock class="q-mt-md">
        <template #left>
          <div class="text-weight-bold q-mb-sm text-grey-3">Borrar archivo simple:</div>
          <CodeBlock lang="bash" content="rm archivo_feo.txt" />
        </template>
        <template #right>
          <div class="text-weight-bold q-mb-sm text-grey-3">
            Borrar carpeta (Recursivo + Force):
          </div>
          <CodeBlock lang="bash" content="rm -rf carpeta_fea/" />
        </template>
      </SplitBlock>
    </div>

    <!-- RETO -->
    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>🏆 Reto de Limpieza</SectionTitle>
      <TextBlock>
        Vamos a practicar creando un desastre y luego limpiándolo. Ejecuta estos comandos uno por
        uno:
      </TextBlock>

      <StepsBlock
        :steps="[
          'Crear una carpeta llamada temp: <code>mkdir temp</code>',
          'Entrar en ella: <code>cd temp</code>',
          'Crear 3 archivos: <code>touch a.txt b.txt c.txt</code>',
          'Renombrar a.txt a final.txt: <code>mv a.txt final.txt</code>',
          'Borrar los otros dos: <code>rm b.txt c.txt</code>',
          'Salir de la carpeta: <code>cd ..</code>',
          'Borrar la carpeta entera: <code>rm -rf temp</code>',
        ]"
      />
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
/* ESTILOS BASE */
.intro-hero,
.section-group {
  width: 100%;
  max-width: 1100px;
  margin: 0 auto 3.5rem auto;
}

/* HERO (Color Verde para Módulo 0) */
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

/* CARDS ESPECÍFICAS (MV) */
.concept-card {
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 16px;
  padding: 24px;
  height: 100%;
  transition: transform 0.2s;
}

.concept-card:hover {
  background: rgba(30, 41, 59, 0.6);
  border-color: rgba(56, 189, 248, 0.3);
}

.concept-card p {
  color: #94a3b8;
  font-size: 0.95rem;
  margin-bottom: 12px;
  line-height: 1.5;
}

/* RESPONSIVE */
@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
  .intro-hero {
    padding: 2rem 1rem;
  }
}
</style>
