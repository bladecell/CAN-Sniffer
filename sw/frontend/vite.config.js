import { defineConfig } from 'vite'
import { svelte } from '@sveltejs/vite-plugin-svelte'
import { viteSingleFile } from "vite-plugin-singlefile"

export default defineConfig({
	plugins: [
		svelte(),
		viteSingleFile(),
	],
	build: {
		reportCompressedSize: false,
		cssCodeSplit: false,
		assetsInlineLimit: 100000000, // This will inline the favicon automatically
	},
	server: {
		proxy: {
			'/api': {
				target: 'http://can-sniffer.local',
				changeOrigin: true,
				rewrite: (path) => path
			}
		}
	}
})