<template>
  <section class="intro-hero self-stretch q-mb-xl" :class="themeClass">
    <div class="hero-content">
      <!-- Breadcrumbs -->
      <div
        class="text-overline text-accent text-weight-bold q-mb-sm flex items-center justify-center fade-in-up"
      >
        <q-icon :name="icon || 'school'" size="xs" class="q-mr-xs" />
        {{ moduleTitle }}
        <span class="q-mx-sm text-grey-6">/</span>
        {{ title }}
      </div>

      <!-- Título Principal -->
      <h1 class="hero-title fade-in-up delay-100">
        {{ title }}
      </h1>

      <!-- Subtítulo -->
      <p class="hero-subtitle fade-in-up delay-200">
        {{ description }}
      </p>
    </div>

    <!-- Fondo decorativo -->
    <div class="hero-grid-bg"></div>
  </section>
</template>

<script setup lang="ts">
import { computed } from 'vue';

const props = defineProps<{
  title: string;
  description: string;
  moduleTitle: string;
  moduleId: string; // ej: 'modulo-0'
  icon?: string;
}>();

// Lógica de colores basada en el módulo
const themeClass = computed(() => {
  if (props.moduleId.includes('modulo-0')) return 'theme-green';
  if (props.moduleId.includes('modulo-1')) return 'theme-purple';
  if (props.moduleId.includes('modulo-2')) return 'theme-blue';
  if (props.moduleId.includes('modulo-4')) return 'theme-red';
  if (props.moduleId.includes('modulo-8')) return 'theme-gold';
  return 'theme-default';
});
</script>

<style scoped>
/* (Copia aquí EXACTAMENTE los estilos CSS de .intro-hero, .hero-title, etc. de tu archivo anterior) */
/* ... CSS ... */
</style>
