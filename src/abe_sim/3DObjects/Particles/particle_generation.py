import shutil

particleNames = [
    ["creamed_butter","CreamedButter"],
    ["cocoa_flour_mixture","CocoaFlourMixture"],
    ["cocoa_flour_butter_mixture","CocoaFlourButterMixture"],
    ["cocoa_flour_butter_cornflakes_mixture","CocoaFlourButterCornflakesMixture"],
    ["icing","Icing"],
    ["almondcookie2_beaten_mixture","AlmondCookie2BeatenMixture"],
    ["almondcookie2_intermediate_mixture","AlmondCookie2IntermediateMixture"],
    ["almondcookie2_dough","AlmondCookie2Dough"],
    ["almondcookie3_beaten_mixture","AlmondCookie3BeatenMixture"],
    ["almondcookie3_intermediate_mixture","AlmondCookie3IntermediateMixture"],
    ["almondcookie3_dough","AlmondCookie3Dough"],
    ["almondcookie4_beaten_mixture","AlmondCookie4BeatenMixture"],
    ["almondcookie4_dough","AlmondCookie4Dough"],
    ["almondcookie5_beaten_mixture","AlmondCookie5BeatenMixture"],
    ["almondcookie5_intermediate_mixture","AlmondCookie5IntermediateMixture"],
    ["almondcookie5_dough","AlmondCookie5Dough"],
    ["almondcookie_beaten_mixture","AlmondCookieBeatenMixture"],
    ["almondcookie_intermediate_mixture","AlmondCookieIntermediateMixture"],
    ["almondcookie_dough","AlmondCookieDough"],
    ["chicken_salad_sauce_mixture","ChickenSaladSauceMixture"],
    ["brownie_beaten_mixture","BrownieBeatenMixture"],
    ["brownie_egg_sugar_mixture","BrownieEggSugarMixture"],
    ["brownie_flour_sugar_mixture","BrownieFlourSugarMixture"],
    ["brownie_dough","BrownieDough"],
    ["bisquick_shortcake_intermediate_mixture","BisquickShortcakeIntermediateMixture"],
    ["bisquick_shortcake_dough","BisquickShortcakeDough"],
    ["black_Bean_Salad2_dressing","BlackBeanSalad2Dressing"],
    ["black_Bean_Salad2_stirred_mixture","BlackBeanSalad2StirredMixture"],
    ["black_Bean_Salad4_liquid_mixture","BlackBeanSalad4LiquidMixture"],
    ["broccoli_salad_dressing","BroccoliSaladDressing"],
    ["cupcakes_intermediate_mixture","CupcakesIntermediateMixture"],
    ["chocolate_fudge_cookies_stirred_mixture","ChocolateFudgeCookiesStirredMixture"],
    ["classic_greek_salad_dressing","ClassicGreekSaladDressing"],
    ["classic_potato_salad_dressing","ClassicPotatoSaladDressing"],
    ["coconut_tuiles_beaten_butter","CoconutTuilesBeatenButter"],
    ["coconut_tuiles_butter_sugar_mixture", "CoconutTuilesButterSugarMixture"],
    ["coconut_tuiles_beaten_egg_mixture", "CoconutTuilesBeatenEggMixture"],
    ["cole_slaw_dressing", "ColeSlawDressing"],
    ["croutons_vinegar_salad_dressing", "CroutonsVinegarSaladDressing"],
    ["cucumber_slices_liquid_mixture", "CucumberSlicesLiquidMixture"],
    ["mashed_banana", "MashedBanana"],
    ["banana_bread_creamed_mixture", "BananaBreadCreamedMixture"],
    ["banana_bread_beaten_mixture", "BananaBreadBeatenMixture"],
    ["banana_bread_batter", "BananaBreadBatter"],
    ["cherry_tomato_salad_dressing", "CherryTomatoSaladDressing"],
    ["oatmeal_cookies_sifted_ingredients", "OatmealCookiesSiftedIngredients"],
    ["oatmeal_cookies_oats_mixture", "OatmealCookiesOatsMixture"],
    ["oatmeal_cookies_dough", "OatmealCookiesDough"],
    ["mexican_wedding_cookies_dough", "MexicanWeddingCookiesDough"],
    ["black_bean_sweet_potato_mixture", "BlackBeanSweetPotatoMixture"],
    ["ginger_snaps_creamed_butter", "GingerSnapsCreamedButter"],
    ["ginger_snaps_cream_eggs_molasses_mix", "GingerSnapsCreamEggsMolassesMix"],
    ["ginger_snaps_dry_mixture", "GingerSnapsDryMixture"],
    ["ginger_snaps_dough", "GingerSnapsDough"]
    ]
for particle in particleNames:
    shutil.copy("butter_particle.mtl", particle[0] + ".mtl")

    f = open(particle[0] + ".obj", "a")
    objContent = ""
    objContent += "# Blender v2.78 (sub 0) OBJ File: ''\n# www.blender.org\nmtllib butter_particle.mtl\no Cube_Cube.001\nv 0.010000 0.000000 0.010000\nv 0.010000 0.000000 -0.010000\nv 0.000000 0.014000 0.000000\nv 0.000000 -0.014000 0.000000\nv -0.010000 0.000000 -0.010000\nv -0.010000 0.000000 0.010000\nvn 0.8137 0.5812 0.0000\nvn 0.0000 -0.5812 -0.8137\nvn -0.8137 -0.5812 0.0000\nvn 0.8137 -0.5812 0.0000\nvn 0.0000 -0.5812 0.8137\nvn 0.0000 0.5812 -0.8137\nvn -0.8137 0.5812 0.0000\nvn 0.0000 0.5812 0.8137\nusemtl orange.002\ns 1\nf 1//1 2//1 3//1\nf 4//2 5//2 2//2\nf 5//3 4//3 6//3\nf 2//4 1//4 4//4\nf 1//5 6//5 4//5\nf 2//6 5//6 3//6\nf 6//7 3//7 5//7\nf 1//8 3//8 6//8"
    objContent = objContent.replace("butter_particle", particle[0])
    f.write(objContent)
    f.close()

    f = open(particle[0] + ".urdf", "a")
    urdfContent = ""
    urdfContent += "<?xml version=\"1.0\"?>\n<robot name=\"butter_particle\">\n  <link name=\"butter_particle\">\n    <inertial>\n        <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n        <mass value=\"0.01\"/>\n        <inertia ixx=\"2e-5\" ixy=\"0\" ixz=\"0\" iyy=\"2e-5\" iyz=\"0\" izz=\"2e-5\"/>\n    </inertial>\n    <visual>\n      <geometry>\n        <mesh filename=\"butter_particle.obj\"/>\n      </geometry>\n    </visual>\n    <collision>\n      <geometry>\n        <mesh filename=\"butter_particle.obj\"/>\n      </geometry>\n    </collision>\n  </link>\n</robot>"
    urdfContent = urdfContent.replace("butter_particle", particle[0])
    f.write(urdfContent)
    f.close()

    f = open("objectknowledge_copypaste.txt", "a")
    txtContent = ""
    txtContent += "{\n              \"type\": \"" + particle[1] + "\",\n              \"simtype\": \"ktree\",\n              \"filename\": \"3DObjects/Particles/butter_particle.urdf\",\n              \"fn\": {\n                     \"absorbable\": true,\n                     \"consumable\": true,\n                     \"transportable\": true,\n                     \"mixable\": true,\n                     \"sticky\": true,\n                     \"temperatureUpdateable\": true,\n                     \"mixing\": {\n                            \"mixableRadius\": 0.4,\n                            \"substanceRadius\": 0.4\n                     }\n              },\n              \"customStateVariables\": {\n                     \"mixing\": {\n                            \"hp\": 720\n                     }\n              }\n       },"
    txtContent = txtContent.replace("butter_particle", particle[0])
    f.write(txtContent)
    f.close()
