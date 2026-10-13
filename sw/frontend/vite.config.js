import { defineConfig } from 'vite'
import { svelte } from '@sveltejs/vite-plugin-svelte'
import { viteSingleFile } from "vite-plugin-singlefile"
import path from 'path';
import { execFileSync } from 'node:child_process';
import { readFileSync, writeFileSync, rmSync } from 'node:fs';
import { gzipSync } from 'node:zlib';

// 7-Zip's deflate beats zlib -9 by ~4%; falls back to zlib when 7z is missing.
const gzipIndex = () => ({
	name: 'gzip-index',
	apply: 'build',
	closeBundle() {
		const html = path.resolve(__dirname, 'dist/index.html');
		const gz = html + '.gz';
		try {
			rmSync(gz, { force: true });
			execFileSync('7z', ['a', '-tgzip', '-mx=9', '-mpass=15', '-mfb=258', gz, html], { stdio: 'ignore' });
		} catch {
			writeFileSync(gz, gzipSync(readFileSync(html), { level: 9 }));
		}
		rmSync(html);
	}
});

export default defineConfig({
	plugins: [
		svelte(),
		viteSingleFile(),
		gzipIndex()
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
		host: true,

		proxy: {
			'/api': {
				target: 'http://can-sniffer.local',
			},
			'/ws': {
				target: 'ws://can-sniffer.local',
				ws: true,
				secure: false,
			}
		}
	}
})
