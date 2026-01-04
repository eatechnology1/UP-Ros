<template>
  <div class="code-block-wrapper">
    <!-- Header -->
    <div v-if="title" class="code-header">
      <span class="code-title">
        {{ title }}
        <span v-if="lang" class="code-lang">{{ lang }}</span>
      </span>
      <div class="code-actions">
        <q-btn
          v-if="copyable"
          dense
          flat
          round
          size="sm"
          icon="content_copy"
          @click="copyToClipboard"
          aria-label="Copiar código"
        />
        <q-btn
          v-if="expandable"
          dense
          flat
          round
          size="sm"
          :icon="expanded ? 'expand_less' : 'expand_more'"
          @click="toggleExpanded"
        />
      </div>
    </div>

    <!-- Contenido -->
    <div class="code-container" :class="{ 'code-expanded': expanded }">
      <pre><code>{{ displayContent }}</code></pre>
    </div>

    <!-- Feedback copia -->
    <q-banner v-if="copied" class="code-copied-banner" inline>
      ¡Código copiado al portapapeles!
    </q-banner>
  </div>
</template>

<script setup lang="ts">
import { ref, computed, onMounted, nextTick } from 'vue';

interface Props {
  title?: string;
  lang?: string;
  copyable?: boolean;
  expandable?: boolean;
  content?: string;
}

const props = withDefaults(defineProps<Props>(), {
  copyable: true,
  expandable: false,
  content: '',
});

const expanded = ref(false);
const copied = ref(false);

const displayContent = computed(() => {
  return props.content || '';
});

function toggleExpanded() {
  expanded.value = !expanded.value;
}

async function copyToClipboard() {
  if (!navigator.clipboard || !props.content) return;

  try {
    await navigator.clipboard.writeText(props.content);
    copied.value = true;

    await nextTick();
    setTimeout(() => {
      copied.value = false;
    }, 2000);
  } catch (err) {
    console.error('Error copying code:', err);
  }
}

onMounted(() => {
  if (props.content && props.content.length > 800 && !props.expandable) {
    expanded.value = true;
  }
});
</script>

<style scoped>
.code-block-wrapper {
  max-width: 900px;
  margin: 32px auto;
  border-radius: 16px;
  overflow: hidden;
  backdrop-filter: blur(14px);
  -webkit-backdrop-filter: blur(14px);
  background: rgba(15, 23, 42, 0.95);
  border: 1px solid rgba(148, 163, 184, 0.3);
  box-shadow:
    0 16px 48px rgba(15, 23, 42, 0.8),
    inset 0 1px 0 rgba(255, 255, 255, 0.06);
  font-family: 'Fira Code', 'Monaco', 'Consolas', monospace;
  transition: var(--transition-base);
}

.code-block-wrapper:hover {
  transform: translateY(-2px);
  box-shadow:
    0 24px 64px rgba(15, 23, 42, 0.85),
    inset 0 1px 0 rgba(255, 255, 255, 0.08);
}

/* HEADER */
.code-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 12px 20px;
  background: rgba(20, 30, 50, 0.9);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
}

.code-title {
  color: #cbd5f5;
  font-size: 0.875rem;
  font-weight: 500;
  letter-spacing: 0.3px;
}

.code-lang {
  background: rgba(110, 203, 255, 0.2);
  color: #6ecbff;
  padding: 2px 8px;
  border-radius: 6px;
  font-size: 0.75rem;
  margin-left: 8px;
  font-weight: 500;
}

.code-actions {
  display: flex;
  gap: 4px;
}

/* CONTENEDOR */
.code-container {
  max-height: 400px;
  overflow-y: auto;
  overflow-x: auto;
}

.code-expanded {
  max-height: none;
}

pre {
  margin: 0;
  padding: 20px;
  background: transparent;
}

code {
  font-size: 0.875rem;
  line-height: 1.6;
  color: #e6edf7;
  white-space: pre;
  word-break: break-word;
}

/* SCROLLBAR */
.code-container::-webkit-scrollbar {
  width: 8px;
  height: 8px;
}

.code-container::-webkit-scrollbar-track {
  background: rgba(148, 163, 184, 0.1);
  border-radius: 4px;
}

.code-container::-webkit-scrollbar-thumb {
  background: rgba(110, 203, 255, 0.4);
  border-radius: 4px;
}

.code-container::-webkit-scrollbar-thumb:hover {
  background: rgba(110, 203, 255, 0.6);
}

/* COPIED */
.code-copied-banner {
  background: rgba(34, 197, 94, 0.2) !important;
  border: 1px solid rgba(34, 197, 94, 0.4) !important;
  color: #dcfce7 !important;
}

/* RESPONSIVE */
@media (max-width: 768px) {
  .code-block-wrapper {
    margin: 24px 16px;
  }
}
</style>
