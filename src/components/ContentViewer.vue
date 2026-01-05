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
  color: #cbd5e1;
  font-size: 1.05rem;
  line-height: 1.75;
}

.markdown-body :deep(p) {
  margin-bottom: 1.5rem;
}

.markdown-body :deep(h2) {
  color: #f1f5f9;
  font-size: 1.5rem;
  font-weight: 700;
  margin-top: 2.5rem;
  margin-bottom: 1rem;
  border-bottom: 1px solid rgba(148, 163, 184, 0.1);
  padding-bottom: 0.5rem;
}

.markdown-body :deep(h3) {
  color: #e2e8f0;
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
  color: #38bdf8;
  text-decoration: none;
  border-bottom: 1px solid rgba(56, 189, 248, 0.3);
  transition: border-color 0.2s;
}

.markdown-body :deep(a:hover) {
  border-bottom-color: #38bdf8;
}

.markdown-body :deep(code) {
  background: rgba(15, 23, 42, 0.6);
  color: #38bdf8;
  padding: 2px 6px;
  border-radius: 4px;
  font-family: 'Fira Code', monospace;
  font-size: 0.9em;
  border: 1px solid rgba(148, 163, 184, 0.2);
}

.markdown-body :deep(pre) {
  background: rgba(15, 23, 42, 0.95);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 8px;
  padding: 16px;
  overflow-x: auto;
  margin-bottom: 1.5rem;
}

.markdown-body :deep(pre code) {
  background: transparent;
  color: #e2e8f0;
  padding: 0;
  border: none;
  font-size: 0.9rem;
}

.markdown-body :deep(blockquote) {
  border-left: 4px solid #38bdf8;
  background: rgba(56, 189, 248, 0.1);
  margin: 1.5rem 0;
  padding: 1rem 1.5rem;
  border-radius: 0 8px 8px 0;
  font-style: italic;
  color: #e2e8f0;
}
</style>
