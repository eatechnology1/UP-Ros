<template>
  <div class="content-wrapper">
    <h1 class="title">{{ section?.title }}</h1>

    <div v-if="section?.markdown" class="markdown" v-html="renderedMarkdown" />

    <div v-if="section?.images?.length" class="images">
      <q-img
        v-for="img in section.images"
        :key="img"
        :src="`/src/assets/images/${img}`"
        class="content-image"
        fit="contain"
      />
    </div>
  </div>
</template>

<script setup lang="ts">
import { computed } from 'vue';
import { sections } from 'src/data';
import { marked } from 'marked';

const props = defineProps<{ sectionId: string }>();

const section = computed(() => sections[props.sectionId]);

const renderedMarkdown = computed(() =>
  section.value?.markdown ? marked.parse(section.value.markdown) : '',
);
</script>

<style scoped>
.content-wrapper {
  max-width: 900px;
  margin: 0 auto;
  padding-bottom: 40px;
}

.title {
  margin-bottom: 24px;
}

.markdown {
  line-height: 1.7;
}

.markdown h2 {
  margin-top: 32px;
}

.images {
  margin-top: 32px;
  display: flex;
  justify-content: center;
}

.content-image {
  max-width: 600px;
}
</style>
