"""
generation/

Farm generation stages, run in the order documented in CLAUDE.md
"Generation order". Each stage module exposes `run(scene, config, rng) ->
scene`; orchestrator.generate_farm(config) runs them all in sequence.
"""
