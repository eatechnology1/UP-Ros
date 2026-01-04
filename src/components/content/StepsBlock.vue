<template>
  <ol class="steps-block">
    <li v-for="(step, index) in steps" :key="index">
      {{ step }}
    </li>
  </ol>
</template>

<script setup lang="ts">
defineProps<{ steps: string[] }>();
</script>

<style scoped>
.steps-block {
  max-width: 900px;
  margin: 24px auto;
  padding-left: 24px;
}

.steps-block li {
  margin-bottom: 12px;
}
</style>
