<template>
  <div class="split-block">
    <div class="left">
      <slot name="left" />
    </div>
    <div class="right">
      <slot name="right" />
    </div>
  </div>
</template>

<style scoped>
.split-block {
  max-width: 900px;
  margin: 32px auto;
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 24px;
}

@media (max-width: 768px) {
  .split-block {
    grid-template-columns: 1fr;
  }
}
</style>
