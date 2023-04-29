# ChatImproVR template
## Change the package's name in Cargo.toml 
It is very important to give your resources unique names! Pick a **unique** name for your plugin, ideally a long name, and assign it to `name = ""` in `Cargo.toml`. This way you can use the helpful `pkg_namespace!()` macro to assign public names to your resources!


## Preparation
Make sure you have the `wasm32-unknown-unknown` target installed:
```sh
rustup target add wasm32-unknown-unknown
```

## Building
Now you can use `cargo build --release` to build. Your plugin will show up under `target/wasm32-unknown-unknown/release/<cool pkg name>.wasm`.

## Testing
Because `.cargo/config.toml` is set up to compile for the WASM target, tests will fail to run by default. You can compile and run tests using the provided `test_pc` alias:
```sh
cargo test_pc
```
