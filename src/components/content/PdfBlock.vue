<template>
  <div class="pdf-block">
    <q-btn icon="picture_as_pdf" color="primary" :label="label" :href="src" target="_blank" />
  </div>
</template>

<script setup lang="ts">
defineProps<{
  label: string;
  src: string;
}>();
</script>

<style scoped>
.pdf-block {
  max-width: 900px;
  margin: 24px auto;
}
</style>
