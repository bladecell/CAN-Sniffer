if (typeof crypto !== "undefined" && !crypto.randomUUID) {
	crypto.randomUUID = () =>
		"xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx".replace(/[xy]/g, (c) => {
			const r = (Math.random() * 16) | 0;
			return (c === "x" ? r : (r & 0x3) | 0x8).toString(16);
		});
}

import { mount } from 'svelte'
import App from './App.svelte'
import './app.css'

const app = mount(App, {
	target: document.getElementById('app'),
})
export default app