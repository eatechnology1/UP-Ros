<template>
  <q-page class="q-pa-lg column items-center">
    <!-- ========================================================================================
         1. HERO SECTION: EL ENCABEZADO
         * Cambia el título, el span (color) y la descripción.
         * Ajusta el degradado en la clase .intro-hero del CSS si quieres otro color (azul, verde, etc.)
    ======================================================================================== -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <!-- Badge pequeño superior -->
        <div class="text-overline text-accent text-weight-bold q-mb-sm">MÓDULO X: CATEGORÍA</div>

        <!-- Título Principal -->
        <h1 class="hero-title">Título Principal <span class="text-primary">Destacado</span></h1>

        <!-- Descripción corta (El "Hook") -->
        <TextBlock>
          Escribe aquí el resumen enganchador de la clase. Debe explicar qué aprenderán y por qué es
          importante, antes de mostrar cualquier código.
        </TextBlock>
      </div>
    </section>

    <!-- ========================================================================================
         2. CONTENIDO PRINCIPAL
         * Añade tantos <div class="section-group"> como necesites.
    ======================================================================================== -->

    <!-- SECCIÓN A: TEORÍA O INTRODUCCIÓN -->
    <div class="section-group self-stretch">
      <SectionTitle>1. Título de la Sección</SectionTitle>

      <TextBlock>
        Explicación teórica clara. Aquí puedes introducir conceptos clave antes de ir a la práctica.
        Usa listas si es necesario:
        <ul>
          <li><strong>Concepto A:</strong> Definición corta.</li>
          <li><strong>Concepto B:</strong> Definición corta.</li>
        </ul>
      </TextBlock>
    </div>

    <!-- SECCIÓN B: BLOQUE DIVIDIDO (Ideal para Código vs Explicación) -->
    <div class="section-group self-stretch">
      <SectionTitle>2. Ejemplo Práctico</SectionTitle>

      <SplitBlock>
        <template #left>
          <TextBlock> Explica aquí qué hace el código de la derecha. </TextBlock>
          <AlertBlock type="info" title="Tip Pro">
            Un consejo rápido sobre buenas prácticas.
          </AlertBlock>
        </template>

        <template #right>
          <CodeBlock
            title="ejemplo.py"
            lang="python"
            content="print('Hola ROS 2')"
            :copyable="true"
          />
        </template>
      </SplitBlock>
    </div>

    <!-- SECCIÓN C: CIERRE O DIAGRAMA -->
    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>3. Resumen</SectionTitle>
      <TextBlock> Conclusión final de la lección. </TextBlock>
    </div>
  </q-page>
</template>

<script setup lang="ts">
// --- IMPORTACIONES ESTÁNDAR ---
// No borres esto, son los bloques de construcción de tu UI
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
// import StepsBlock from 'components/content/StepsBlock.vue'; // Opcional
// import ImageBlock from 'components/content/ImageBlock.vue'; // Descomentar si usas imágenes

// --- LÓGICA / VARIABLES ---
// Aquí puedes definir const con strings largos de código para mantener el template limpio
</script>

<style scoped>
/* ========================================================================================
   ESTILOS BASE (NO TOCAR)
   Mantienen la alineación y coherencia en todas las páginas.
======================================================================================== */
.intro-hero,
.section-group {
  width: 100%;
  max-width: 1100px; /* Ancho máximo estándar de lectura */
  margin: 0 auto 3.5rem auto; /* Separación vertical entre bloques */
}

/* ========================================================================================
   ESTILO DEL HERO
   * Puedes cambiar el radial-gradient para diferenciar módulos:
     - Azul: rgba(56, 189, 248, 0.15)
     - Verde: rgba(16, 185, 129, 0.15)
     - Morado: rgba(168, 85, 247, 0.15)
======================================================================================== */
.intro-hero {
  padding: 3rem 2rem;
  background:
    radial-gradient(circle at center, rgba(56, 189, 248, 0.15), transparent 60%),
    /* <--- CAMBIAR COLOR AQUÍ */ rgba(15, 23, 42, 0.8);
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

/* RESPONSIVE: Ajustes para móviles */
@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
  .intro-hero {
    padding: 2rem 1rem;
  }
}
</style>
