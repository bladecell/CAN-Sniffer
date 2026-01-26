import { defineConfig } from 'vite'
import { svelte } from '@sveltejs/vite-plugin-svelte'
import { viteSingleFile } from "vite-plugin-singlefile"
import path from 'path';

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
	resolve: {
		alias: {
			$lib: path.resolve(__dirname, './src/lib')
		}
	},
	server: {
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