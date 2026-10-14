import { defineConfig } from 'vite';

export default defineConfig({
  // Relative asset paths, so the build works from any static host sub-path.
  base: './',
  worker: { format: 'es' },
  // Listen on every interface so phones and other machines on the LAN can open it.
  server: { host: '0.0.0.0' },
  preview: { host: '0.0.0.0' },
  build: { target: 'es2022' },
});
