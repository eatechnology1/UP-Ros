<template>
  <router-view />
</template>

<script lang="ts">
import { defineComponent } from 'vue';
import { useTheme } from 'src/composables/useTheme';

export default defineComponent({
  name: 'App',
  setup() {
    // Inicializar el tema desde localStorage al montar la app
    // El watch con { immediate: true } dentro del composable lo aplica automáticamente
    useTheme();
  },
});
</script>
