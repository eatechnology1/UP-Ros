<template>
  <div class="video-block">
    <iframe :src="src" frameborder="0" allowfullscreen />
  </div>
</template>

<script setup lang="ts">
defineProps<{ src: string }>();
</script>

<style scoped>
.video-block {
  max-width: 900px;
  margin: 0 auto 32px auto;
}

iframe {
  width: 100%;
  height: 420px;
}
</style>
