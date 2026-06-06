<template>
  <div class="code-block-wrapper">
    <!-- Header -->
    <div v-if="title || lang" class="code-header">
      <div class="code-title-group">
        <span v-if="title" class="code-title">{{ title }}</span>
        <span v-if="lang" class="code-lang">{{ lang }}</span>
      </div>

      <div class="code-actions">
        <!-- Botón Copiar con Feedback Visual -->
        <q-btn
          v-if="copyable"
          dense
          flat
          round
          size="sm"
          :icon="copied ? 'check' : 'content_copy'"
          :color="copied ? 'positive' : 'grey-4'"
          @click="handleCopy"
          aria-label="Copiar código"
        >
          <q-tooltip v-if="!copied">Copiar</q-tooltip>
          <q-tooltip v-else class="bg-positive">¡Copiado!</q-tooltip>
        </q-btn>

        <!-- Botón Expandir -->
        <q-btn
          v-if="expandable"
          dense
          flat
          round
          size="sm"
          :icon="expanded ? 'expand_less' : 'expand_more'"
          color="grey-4"
          @click="toggleExpanded"
          aria-label="Expandir código"
        >
          <q-tooltip>{{ expanded ? 'Contraer' : 'Expandir' }}</q-tooltip>
        </q-btn>
      </div>
    </div>

    <!-- Contenido -->
    <div class="code-container" :class="{ 'code-expanded': expanded }">
      <pre><code>{{ content }}</code></pre>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, onMounted } from 'vue';
import { copyToClipboard } from 'quasar';

interface Props {
  title?: string;
  lang?: string;
  copyable?: boolean;
  expandable?: boolean;
  content?: string;
}

const props = withDefaults(defineProps<Props>(), {
  copyable: true,
  expandable: false, // Por defecto no expandible, salvo que sea muy largo
  content: '',
});

const expanded = ref(false);
const copied = ref(false);

function toggleExpanded() {
  expanded.value = !expanded.value;
}

function handleCopy() {
  if (!props.content) return;

  copyToClipboard(props.content)
    .then(() => {
      copied.value = true;
      setTimeout(() => {
        copied.value = false;
      }, 2000);
    })
    .catch((err) => {
      console.error('Error al copiar:', err);
    });
}

onMounted(() => {
  // Auto-expandir si el contenido es corto, o colapsar si es muy largo y no se especificó expandable=true
  // Lógica mejorada: Si es muy largo (> 20 líneas aprox o 800 chars), lo mostramos con scroll
  // Si se pasa prop expandable=true, inicia contraído.
  if (props.expandable) {
    expanded.value = false;
  } else {
    // Si no es "expandable" explícitamente, siempre mostramos todo (o con scroll CSS)
    expanded.value = true;
  }
});
</script>

<style scoped>
.code-block-wrapper {
  width: 100%;
  /* Eliminado max-width fijo para adaptabilidad */
  border-radius: 12px;
  overflow: hidden;
  backdrop-filter: blur(14px);
  -webkit-backdrop-filter: blur(14px);
  background: var(--bg-surface-solid);
  border: 1px solid var(--border-medium);
  box-shadow:
    0 4px 6px -1px rgba(0, 0, 0, 0.1),
    0 2px 4px -1px rgba(0, 0, 0, 0.06);
  font-family: 'Fira Code', 'Monaco', 'Consolas', monospace;
  margin-bottom: 1.5rem;
  transition:
    transform 0.2s,
    box-shadow 0.2s;
}

.code-block-wrapper:hover {
  box-shadow: 0 10px 15px -3px rgba(0, 0, 0, 0.2);
}

/* HEADER */
.code-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 8px 16px;
  background: var(--bg-surface-hover);
  border-bottom: 1px solid var(--border-subtle);
}

.code-title-group {
  display: flex;
  align-items: center;
  gap: 12px;
}

.code-title {
  color: var(--text-secondary);
  font-size: 0.85rem;
  font-weight: 600;
  letter-spacing: 0.3px;
}

.code-lang {
  background: var(--bg-code);
  color: var(--text-code);
  padding: 2px 8px;
  border-radius: 4px;
  font-size: 0.7rem;
  font-weight: 600;
  text-transform: uppercase;
  border: 1px solid var(--border-hover);
}

.code-actions {
  display: flex;
  gap: 4px;
}

/* CONTENEDOR */
.code-container {
  /* Altura máxima por defecto para evitar bloques gigantes */
  max-height: 350px;
  overflow-y: auto;
  overflow-x: auto;
  transition: max-height 0.3s ease-out;
}

.code-expanded {
  max-height: none !important; /* Muestra todo si está expandido */
}

pre {
  margin: 0;
  padding: 16px;
  background: transparent;
  min-width: 100%; /* Asegura que el pre llene el contenedor */
}

code {
  font-size: 0.9rem;
  line-height: 1.6;
  color: var(--text-primary); /* Slate-50 */
  white-space: pre; /* Mantiene formato exacto */
  font-family: inherit;
  display: inline-block; /* Permite scroll horizontal si es necesario */
}

/* SCROLLBAR PERSONALIZADO */
.code-container::-webkit-scrollbar {
  width: 8px;
  height: 8px;
}

.code-container::-webkit-scrollbar-track {
  background: var(--bg-surface);
}

.code-container::-webkit-scrollbar-thumb {
  background: var(--border-medium);
  border-radius: 4px;
}

.code-container::-webkit-scrollbar-thumb:hover {
  background: var(--border-hover);
}

/* RESPONSIVE */
@media (max-width: 600px) {
  .code-header {
    padding: 8px 12px;
  }

  code {
    font-size: 0.8rem;
  }
}
</style>
