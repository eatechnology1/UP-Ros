<template>
  <div class="text-block" :class="{ 'is-highlighted': highlighted }">
    <div class="text-content">
      <slot />
    </div>
  </div>
</template>

<script setup lang="ts">
defineProps<{
  highlighted?: boolean;
}>();
</script>

<style scoped>
.text-block {
  width: 100%;
  /* Eliminamos márgenes externos para que el padre (SplitBlock o Section) controle el espacio */
  position: relative;
  border-radius: 12px;
  background: transparent; /* Por defecto transparente para integrarse mejor */
  transition: all 0.3s ease;
}

/* Tipografía Base */
.text-content {
  color: var(--text-secondary);
  font-size: 1.05rem;
  line-height: 1.75;
  letter-spacing: 0.01em;
}

/* =========================================
   ESTILOS INTERNOS (Rich Text)
   Usamos :deep() para afectar al HTML inyectado en el slot
========================================= */

/* Negritas */
.text-block :deep(strong),
.text-block :deep(b) {
  color: var(--text-primary);
  font-weight: 700;
}

/* Cursivas */
.text-block :deep(em),
.text-block :deep(i) {
  color: var(--text-muted);
  font-style: italic;
}

/* Código en línea */
.text-block :deep(code) {
  background: var(--bg-code);
  color: var(--text-code);
  padding: 2px 6px;
  border-radius: 4px;
  font-size: 0.9em;
  font-family: 'Fira Code', 'Monaco', monospace;
  border: 1px solid var(--border-hover);
}

/* Enlaces */
.text-block :deep(a) {
  color: var(--text-link);
  text-decoration: none;
  border-bottom: 1px solid var(--border-hover);
  transition: all 0.2s;
}

.text-block :deep(a:hover) {
  color: var(--text-link-hover);
  border-bottom-color: var(--text-link-hover);
}

/* Listas desordenadas simples (dentro de un párrafo) */
.text-block :deep(ul) {
  padding-left: 20px;
  margin: 12px 0;
}

.text-block :deep(li) {
  margin-bottom: 6px;
  list-style-type: disc;
}

/* =========================================
   MODO HIGHLIGHTED (Destacado)
========================================= */
.is-highlighted {
  background: var(--bg-surface);
  border: 1px solid var(--border-medium);
  padding: 24px;
  border-left: 4px solid var(--text-code);
  box-shadow: 0 4px 20px var(--shadow-sm);
}

.is-highlighted .text-content {
  color: var(--text-primary);
}

/* RESPONSIVE */
@media (max-width: 600px) {
  .text-content {
    font-size: 1rem;
    line-height: 1.6;
  }

  .is-highlighted {
    padding: 16px;
  }
}
</style>
