# Safety helmet models — browse and choose

The viewer can use either the **built-in white hard hat** (default, no download) or a **GLB file** you pick.

## How to use your choice

1. Open links below and preview in the browser (Sketchfab / Poly Pizza embed viewers).
2. Download as **GLB** (preferred) or GLTF.
3. Save the file as `www/models/helmet.glb` in this project.
4. In `www/index.html`, set:
   ```javascript
   const HELMET_GLB = 'models/helmet.glb';
   ```
5. Regenerate embedded HTML (from repo root):
   ```bash
   python3 scripts/embed_index_html.py
   ```
   Or ask in chat to “embed index.html” — then `pio run -t upload`.

For testing without reflashing ESP, serve `www/` locally:
```bash
cd www && python3 -m http.server 8080
```
Open `http://localhost:8080/`.

---

## Recommended — construction / safety helmets

| Model | Preview | Notes |
|-------|---------|--------|
| **Hard hat** (Mylom) | [Sketchfab](https://sketchfab.com/3d-models/hard-hat-96b56289116a4cf889ef4ee176fd4c53) | Small (~562 tris), CC Attribution, good fit |
| **Safety helmet** (zilber) | [Sketchfab](https://sketchfab.com/3d-models/safety-helmet-af91164853504bc1ad7adcc9c73ec531) | ~2.4k tris, CC Attribution |
| **Hard Hat Scan** (EFX) | [Sketchfab](https://sketchfab.com/3d-models/hard-hat-scan-medpoly-10edb2bd02444f3a8a3067b9da7ac2d5) | More detail + textures, CC Attribution |
| **Hard Hat** (neutralize) | [Sketchfab](https://sketchfab.com/models/9409fe92cd484e538487d7bed5ff252e/embed) | Simple hard hat |
| **Hard hat** (Google Poly) | [Poly Pizza](https://poly.pizza/m/9AMKar2NlkX) | Low poly, CC Attribution, download GLTF |
| **Hardhat** (Google Poly) ✅ **in use** | [Poly Pizza](https://poly.pizza/m/cyjbHPC6QAM) | Very low poly, CC Attribution |

## More places to search

- [Sketchfab — hard hat](https://sketchfab.com/search?features=downloadable&licenses=322a749bcfa841b29dff14e67a3fe359&type=models&q=hard+hat)
- [Sketchfab — safety helmet](https://sketchfab.com/search?features=downloadable&licenses=322a749bcfa841b29dff14e67a3fe359&type=models&q=safety+helmet)
- [Poly Pizza — helmet](https://poly.pizza/search/helmet)

## Tips

- Prefer **GLB** (single file) over GLTF + separate textures.
- Keep file **under ~2 MB** if you ever want it on ESP flash later; large models load from CDN or your PC dev server only.
- **White helmet:** pick a white/yellow model, or we can force white in code after load.
- Send the **Sketchfab/Poly Pizza link** or drop `helmet.glb` in `www/models/` and say “use this one” — it can be wired in for you.

## License

Many Sketchfab “free download” models are **CC Attribution** — keep credit in docs if you ship the model file in the repo.
