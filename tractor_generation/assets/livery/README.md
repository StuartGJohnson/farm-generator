# Tractor paint maps

Created with the built-in image generation tool for this repository. These are
fixed assets: normal tractor generation never calls an image service. Source
PNGs are packaged with `tractor_generation` and copied into each USD export's
`textures/` directory. UVs repeat at a 2-meter pitch; box faces have planar
projection seams at their boundaries rather than a continuous unwrapped skin.

## Prompts

Tiger generation:

> Use case: stylized-concept. Asset type: square seamless base-color texture for tractor painted bodywork, not a picture of a tractor. Create an edge-to-edge flat tiger stripe pattern: warm saturated orange ground with irregular tapered charcoal-black tiger stripes, generally vertical, with branching organic pointed tips. About 10 major stripes across the square. Crisp clean painted graphic, no fur, no lighting, no shadows, no gradients, no text, no border, no objects. Tile seamlessly on all edges. 1024 by 1024.

Tiger final edit:

> Edit this tractor paint texture. Preserve the dark tiger stripe motif exactly, but fill ALL transparent background with solid saturated tiger orange #E98A23. The entire square must be fully opaque, alpha 255 everywhere. This is a flat albedo paint map, not a transparent decal. No transparency, no shadows, no text.

Cheetah generation:

> Use case: stylized-concept. Asset type: square seamless base-color texture for tractor painted bodywork, not a picture of a tractor. Create an edge-to-edge flat cheetah spot pattern: warm golden ochre ground with scattered solid charcoal-black irregular small rounded spots, varying slightly in size and orientation, roughly 80 spots over the square. True cheetah solid spots, no leopard rosettes. Crisp clean painted graphic, no fur, no lighting, no shadows, no gradients, no text, no border, no objects. Tile seamlessly on all edges. 1024 by 1024.

Cheetah final edit:

> Edit this tractor paint texture. Preserve the dark cheetah spot motif exactly, but fill ALL transparent background with solid golden ochre #DDB45E. The entire square must be fully opaque, alpha 255 everywhere. This is a flat albedo paint map, not a transparent decal. No transparency, no shadows, no text.
