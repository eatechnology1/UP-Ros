<template>
  <div class="split-block">
    <!-- Columna Izquierda -->
    <div class="split-column left-column">
      <slot name="left" />
    </div>

    <!-- Columna Derecha -->
    <div class="split-column right-column">
      <slot name="right" />
    </div>
  </div>
</template>

<script setup lang="ts">
// Estructuralmente simple, no requiere props por ahora
</script>

<style scoped>
.split-block {
  width: 100%;
  display: grid;
  /* minmax(0, 1fr) es CRÍTICO para evitar desbordes con CodeBlocks o tablas */
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 32px; /* Espacio generoso entre columnas */
  align-items: start; /* Alineación superior, evita estiramientos feos */
}

.split-column {
  display: flex;
  flex-direction: column;
  /* Esto asegura que si metes varios bloques (ej: Texto + Alerta) tengan separación automática */
  gap: 24px;
}

/* RESPONSIVE */
@media (max-width: 768px) {
  .split-block {
    grid-template-columns: 1fr; /* Stack vertical */
    gap: 24px;
  }
}
</style>
