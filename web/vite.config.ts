import { defineConfig } from 'vite';

export default defineConfig({
  // Relative asset paths, so the build works from any static host sub-path.
  base: './',
  worker: { format: 'es' },
  build: { target: 'es2022' },
});
