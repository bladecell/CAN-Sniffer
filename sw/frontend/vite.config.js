import { defineConfig } from 'vite'
import { svelte } from '@sveltejs/vite-plugin-svelte'
import tailwindcss from '@tailwindcss/vite'
import { viteSingleFile } from "vite-plugin-singlefile"
import path from 'path';

export default defineConfig({
	plugins: [
		tailwindcss(), // <-- 2. Add it to the plugins array
		svelte(),
		viteSingleFile(),
	],
	build: {
		reportCompressedSize: false,
		cssCodeSplit: false,
		assetsInlineLimit: 100000000,
	},
	resolve: {
		alias: {
			$lib: path.resolve(__dirname, './src/lib')
		}
	},
	server: {
		host: true,
		proxy: {
			'/api': {
				target: 'http://can-sniffer.local',
				changeOrigin: true,
			},
			'/ws': {
				target: 'ws://can-sniffer.local',
				changeOrigin: true,
				ws: true,
				secure: false,
			}
		}
	}
})