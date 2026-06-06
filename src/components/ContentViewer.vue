<template>
  <div class="content-wrapper">
    <transition name="fade" mode="out-in">
      <div :key="sectionId" class="content-inner">
        <!-- 1. Título usando tu componente estándar -->
        <SectionTitle v-if="section?.title">
          {{ section.title }}
        </SectionTitle>

        <!-- 2. Contenido Markdown Estilizado -->
        <div v-if="renderedMarkdown" class="markdown-body" v-html="renderedMarkdown" />

        <!-- 3. Galería de Imágenes -->
        <div v-if="section?.images?.length" class="image-gallery">
          <div v-for="(img, index) in section.images" :key="index" class="gallery-item">
            <ImageBlock :src="getImageUrl(img)" :caption="`Figura ${index + 1}`" :zoomable="true" />
          </div>
        </div>
      </div>
    </transition>
  </div>
</template>

<script setup lang="ts">
import { computed } from 'vue';
import { sections } from 'src/data';
import { marked } from 'marked';
import SectionTitle from 'components/content/SectionTitle.vue';
import ImageBlock from 'components/content/ImageBlock.vue';

const props = defineProps<{ sectionId: string }>();

const section = computed(() => sections[props.sectionId] || null);

const renderedMarkdown = computed(() =>
  section.value?.markdown ? marked.parse(section.value.markdown) : '',
);

function getImageUrl(filename: string) {
  try {
    return new URL(`../assets/images/${filename}`, import.meta.url).href;
  } catch (e) {
    // FIX: Usamos 'e' para satisfacer al linter y ver el error real
    console.error('Error cargando imagen:', filename, e);
    return '';
  }
}
</script>

<style scoped>
.content-wrapper {
  width: 100%;
  max-width: 1100px;
  margin: 0 auto;
  padding-bottom: 60px;
}

/* TRANSICIONES */
.fade-enter-active,
.fade-leave-active {
  transition: opacity 0.3s ease;
}

.fade-enter-from,
.fade-leave-to {
  opacity: 0;
}

/* GALERÍA */
.image-gallery {
  margin-top: 40px;
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 24px;
}

/* =========================================
   ESTILOS MARKDOWN
========================================= */

.markdown-body {
  color: var(--text-secondary);
  font-size: 1.05rem;
  line-height: 1.75;
}

.markdown-body :deep(p) {
  margin-bottom: 1.5rem;
}

.markdown-body :deep(h2) {
  color: var(--text-primary);
  font-size: 1.5rem;
  font-weight: 700;
  margin-top: 2.5rem;
  margin-bottom: 1rem;
  border-bottom: 1px solid var(--border-subtle);
  padding-bottom: 0.5rem;
}

.markdown-body :deep(h3) {
  color: var(--text-secondary);
  font-size: 1.25rem;
  font-weight: 600;
  margin-top: 2rem;
  margin-bottom: 0.75rem;
}

.markdown-body :deep(ul),
.markdown-body :deep(ol) {
  margin-bottom: 1.5rem;
  padding-left: 1.5rem;
}

.markdown-body :deep(li) {
  margin-bottom: 0.5rem;
}

.markdown-body :deep(a) {
  color: var(--text-link);
  text-decoration: none;
  border-bottom: 1px solid var(--border-hover);
  transition: border-color 0.2s;
}

.markdown-body :deep(a:hover) {
  color: var(--text-link-hover);
  border-bottom-color: var(--text-link-hover);
}

.markdown-body :deep(code) {
  background: var(--bg-code);
  color: var(--text-code);
  padding: 2px 6px;
  border-radius: 4px;
  font-family: 'Fira Code', monospace;
  font-size: 0.9em;
  border: 1px solid var(--border-subtle);
}

.markdown-body :deep(pre) {
  background: var(--bg-surface);
  border: 1px solid var(--border-subtle);
  border-radius: 8px;
  padding: 16px;
  overflow-x: auto;
  margin-bottom: 1.5rem;
}

.markdown-body :deep(pre code) {
  background: transparent;
  color: var(--text-secondary);
  padding: 0;
  border: none;
  font-size: 0.9rem;
}

.markdown-body :deep(blockquote) {
  border-left: 4px solid var(--border-hover);
  background: var(--bg-code);
  margin: 1.5rem 0;
  padding: 1rem 1.5rem;
  border-radius: 0 8px 8px 0;
  font-style: italic;
  color: var(--text-secondary);
}
</style>
