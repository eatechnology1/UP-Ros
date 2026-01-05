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
  color: #cbd5e1; /* Slate-300: Color estándar para texto en fondo oscuro */
  font-size: 1.05rem; /* Un poco más grande que el estándar para legibilidad */
  line-height: 1.75; /* Espaciado generoso entre líneas */
  letter-spacing: 0.01em;
}

/* =========================================
   ESTILOS INTERNOS (Rich Text)
   Usamos :deep() para afectar al HTML inyectado en el slot
========================================= */

/* Negritas */
.text-block :deep(strong),
.text-block :deep(b) {
  color: #f1f5f9; /* Slate-100 (Casi blanco) */
  font-weight: 700;
}

/* Cursivas */
.text-block :deep(em),
.text-block :deep(i) {
  color: #94a3b8; /* Slate-400 */
  font-style: italic;
}

/* Código en línea */
.text-block :deep(code) {
  background: rgba(56, 189, 248, 0.15); /* Sky-400 muy suave */
  color: #38bdf8; /* Sky-400 */
  padding: 2px 6px;
  border-radius: 4px;
  font-size: 0.9em;
  font-family: 'Fira Code', 'Monaco', monospace;
  border: 1px solid rgba(56, 189, 248, 0.2);
}

/* Enlaces */
.text-block :deep(a) {
  color: #60a5fa; /* Blue-400 */
  text-decoration: none;
  border-bottom: 1px solid rgba(96, 165, 250, 0.3);
  transition: all 0.2s;
}

.text-block :deep(a:hover) {
  color: #93c5fd; /* Blue-300 */
  border-bottom-color: #93c5fd;
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
  background: rgba(30, 41, 59, 0.4); /* Slate-800 semi-transparente */
  border: 1px solid rgba(148, 163, 184, 0.2);
  padding: 24px;
  border-left: 4px solid #38bdf8; /* Borde izquierdo Cyan */
  box-shadow: 0 4px 20px rgba(0, 0, 0, 0.2);
}

.is-highlighted .text-content {
  color: #e2e8f0; /* Texto ligeramente más brillante */
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
