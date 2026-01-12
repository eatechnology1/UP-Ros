<template>
  <span
    v-if="inline"
    class="math-formula math-inline"
    v-html="renderMath()"
    :class="{ 'has-error': error }"
  ></span>
  <div
    v-else
    class="math-formula math-block"
    v-html="renderMath()"
    :class="{ 'has-error': error }"
  ></div>
</template>

<script setup lang="ts">
import { ref } from 'vue';
import katex from 'katex';
import 'katex/dist/katex.min.css';

const props = defineProps({
  formula: {
    type: String,
    required: true,
  },
  inline: {
    type: Boolean,
    default: false,
  },
});

const error = ref(false);

const renderMath = () => {
  try {
    error.value = false;
    return katex.renderToString(props.formula, {
      throwOnError: false,
      displayMode: !props.inline,
      output: 'html', // Generar solo HTML para evitar problemas de fuentes
    });
  } catch (e) {
    console.error('KaTeX Error:', e);
    error.value = true;
    return `<span class="katex-error">${props.formula}</span>`;
  }
};
</script>

<style>
/* Importación global de estilos de KaTeX asegurada */
@import 'katex/dist/katex.min.css';

.math-formula {
  font-size: 1.1em;
}

.math-block {
  display: flex;
  justify-content: center;
  margin: 1rem 0;
  overflow-x: auto;
  overflow-y: hidden;
}

.katex-error {
  color: #ef4444;
  font-family: monospace;
}

/* Ajustes para modo oscuro si es necesario */
.body--dark .katex {
  color: inherit;
}
</style>
