# NVIDIA SimReady assets used in the simulation scene

The shelf / table scene uses these 3D assets from the Hugging Face dataset
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets) by NVIDIA Corporation,
licensed under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). They are not stored in this repository:
`fetch_assets.py` downloads them into `scenes/assets/simready/`. Changes made when they are placed in our
scene (not to the files): product masses replaced by our own estimates, collision shapes simplified (convex hulls
for products, static meshes for furniture), furniture made static, the crate's hinges locked, the chocolate bar laid flat.

| Asset | Use |
|---|---|
| `rack_f04` | shelf |
| `storage_shelf_basic_b01` | shelf |
| `table_metal_stainless_a03` | table |
| `storage_basket_plastic_a01` | basket |
| `food_processed_boxedFood_b01` | product (box) |
| `food_processed_boxedFood_c01` | product (box) |
| `food_processed_boxedFood_d01` | product (box) |
| `food_box_cracker_a01` | product (box) |
| `food_processed_boxedCereal_l01` | product (box) |
| `food_processed_yogurt_a01` | product (round) |
| `food_processed_yogurt_c01` | product (round) |
| `food_processed_cannedFood_h01` | product (round) |
| `food_beverage_coffeeCup_b01` | product (round) |
| `food_processed_milk_b01` | product (box) |
| `food_processed_milk_c01` | product (round) |
| `food_processed_ketchup_c01` | product (bottle) |
| `food_processed_ketchup_i01` | product (bottle) |
| `food_condiment_ketchup_a02` | product (bottle) |
| `food_processed_chocolate_e01` | product (flat) |
