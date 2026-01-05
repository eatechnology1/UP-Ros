<template>
  <div class="steps-container">
    <ol class="steps-list">
      <li v-for="(step, index) in steps" :key="index" class="step-item">
        <!-- Marcador numérico visual -->
        <div class="step-marker">
          <span class="step-number">{{ index + 1 }}</span>
        </div>

        <!-- Contenido del paso -->
        <div class="step-content">
          {{ step }}
        </div>
      </li>
    </ol>
  </div>
</template>

<script setup lang="ts">
defineProps<{
  steps: string[];
}>();
</script>

<style scoped>
.steps-container {
  width: 100%;
  padding: 8px 0;
}

.steps-list {
  list-style: none;
  margin: 0;
  padding: 0;
  position: relative;
}

.step-item {
  position: relative;
  display: flex;
  gap: 16px;
  margin-bottom: 24px; /* Espacio entre pasos */
}

/* El último item no necesita margen extra inferior */
.step-item:last-child {
  margin-bottom: 0;
}

/* LÍNEA CONECTORA VERTICAL */
/* Dibujamos una línea que conecta los círculos */
.step-item:not(:last-child)::after {
  content: '';
  position: absolute;
  top: 32px; /* Empieza debajo del círculo */
  left: 14px; /* Centrado con el círculo de 28px (14px = mitad) */
  bottom: -20px; /* Se extiende hasta el siguiente */
  width: 2px;
  background: rgba(148, 163, 184, 0.2); /* Slate-400 muy suave */
}

/* MARCADOR (CÍRCULO) */
.step-marker {
  flex-shrink: 0;
  width: 28px;
  height: 28px;
  border-radius: 50%;
  background: linear-gradient(135deg, #38bdf8, #2563eb); /* Sky a Blue */
  display: flex;
  align-items: center;
  justify-content: center;
  box-shadow: 0 0 0 4px rgba(15, 23, 42, 1); /* "Borde" falso del color de fondo para separar la línea */
  z-index: 1; /* Para estar encima de la línea conectora */
  margin-top: 2px; /* Alineación óptica con la primera línea de texto */
}

.step-number {
  color: white;
  font-size: 0.85rem;
  font-weight: 700;
  line-height: 1;
}

/* CONTENIDO TEXTO */
.step-content {
  color: #cbd5e1; /* Slate-300 */
  font-size: 1rem;
  line-height: 1.6;
  padding-top: 2px; /* Pequeño ajuste para alinear con el número */
}

/* RESPONSIVE */
@media (max-width: 600px) {
  .step-item {
    gap: 12px;
  }

  .step-marker {
    width: 24px;
    height: 24px;
    margin-top: 4px;
  }

  .step-number {
    font-size: 0.75rem;
  }

  .step-item:not(:last-child)::after {
    left: 11px; /* Ajuste centro para 24px */
    top: 28px;
  }

  .step-content {
    font-size: 0.95rem;
  }
}
</style>
