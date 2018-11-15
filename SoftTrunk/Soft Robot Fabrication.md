Steps to fabricate the soft trunk

# BoM
Markers
* https://optitrack.com/products/motion-capture-markers/
    * MCP1130 4mm

Rubber
* https://www.reynoldsam.com/product/dragon-skin/

Tubes etc.
* https://www.mcmaster.com/
    * reduced fitting: 5117K56
    * silicone rod: 1249N11
    * silicone tubing: 5236K503

Beeswax
* https://www.amazon.com/Stakich-Pure-White-BEESWAX-Pellets/dp/B0022X1GME/ref=sr_1_1?s=arts-crafts&ie=UTF8&qid=1539636806&sr=1-1&keywords=bleached+beeswax

# silicone casting
1. First assemble the mold, if casting the final arm piece
    1. make sure the wax mold doesn't have rough edges
    ![](img/IMG_1774.JPG)
    above is a picture of a not-so-clean wax mold
    1. assemble them to make final structure (will probably need to shave off fraction of mm into the holes of wax to make them fit)
    ![](img/IMG_1635.JPG)
    assembled mold
    ![](img/IMG_1775.JPG)
    make sure wax pieces form a circle- each piece aligns with the next. They won't align if the wax pieces are deformed.
1. make silicone mix
    1. Silicone to use:
        * 16 ribs: Dragon Skin 30 (actually, maybe better to switch to 10 if too rigid)
        * 19 ribs: Dragon Skin 10
    1. stir ingredients well before mixing (obviously, use separate stirrers!)
    1. **degas**
        1. put in vacuum chamber
        1. turn on pump
        1. twist valve to vacuum air out
        1. and keep turning them on and off to prevent overflowing
        ![](img/IMG_1644.JPG)
        1. do until bubbles subside
        1. turn off pump, slowly pressurize
    1. weigh equal amounts of ingredients(use zero button on scale)
    1. mix well (instructions say to mix for 2-3 minutes)
    1. degas the mixed silicone
1. Spray Ease-Release on mold
![](img/IMG_1642.JPG)
1. pour silicon onto mold
    1. pour slowly, a little below top surface (so it won't overflow in vacuum chamber)
    1. degas
        1. if making final arm piece, degas for ~30 minutes to ensure bubbles are gone
    1. place on desk, pour again and go slightly over the top
    1. slide over with ruler or something to make top flush
1. let it sleep for as long as silicon wants
1. take it out (some technique required)
![](img/IMG_1637.JPG)
    * for the cast for the wax mold, (what good technique is there to release from mold??)
    * for final arm piece, twist it around to loosen it
    ![](img/IMG_1638.JPG)

# wax casting
note that the mold for wax should be about 2%(testing with 1%) larger than what it should be, to account for shrinking
1. warm stuff up
    1. heat oven to 92 degrees Celcius
    1. generate molten wax from block of wax (break up with hammer if necessary)(place so it won't melt to outside cup)
    1. Put silicon mold in oven as well (~15 minutes)
1. align silicon mold with something to make sure it's straight
1. inject wax
    1. take out jar with wax and syringe(use gloves!)
    1. inject to mold, slightly overfilling it
    ![image of mold with wax injected in it](img/IMG_1766.JPG)
    1. syringe will clog if out too long (put back in jar if that happens, to warm it up)
1. make flush
    1. after 10 minutes, use blade to cut wax flush with top surface of mold
    ![image of blade cutting wax](img/IMG_1773.JPG)
1. carefully take them out(once it's not warm anymore, which is about 40 minutes? **Don't wait till they're too cold**- they'll become too brittle and break easily) and handle with care
1. If there's some deforming (twists, bends), try to correct it...

# vanishing wax and cleaning
1. Put arm in oven supported by aluminum shafts, with aluminum plate below to catch molten wax
1. put in boiling water
1. clean with water- run water through holes
1. clean holes for tubing with wax remover, then wipe it off with q-tips(use gloves!)
1. clean holes for tubing with alcohol

# tubing
1. make sure holes are cleaned with wax remover and alcohol
1. put silicon glue on both the hole(use q-tips) and tube(on all the surface that bonds to main piece)- use kimwipes
1. insert
    1. make sure the plugs are flush
    1. insert about 1 cm
    1. 12 inches (30cm) of slack for air tubing
1. Pass tubing through arm

# Glue pieces together


# Using 3D printer
## Solidworks
Export STL
* save as binary, daviation at least 0.0015” or smaller, and 2 degree angle or smaller
* The mold for wax mold is in ConfigurationManager -> Quad Mold Mold

## Stratasys Insight
The softwares Insight and Control Center can be installed from `setup-insight11.2.exe` in DropBox

1. insert STL -> orient STL
1. Click on printer column(right column)
1. pick 0.005" slice height
1. In Modeler setup,
    * part interior style: solid
    * visible surface style: enhanced
    * support style: SMART
1. then "slice"(green flag)
1. then check that it looks fine.("View all layers" button)
1. Finally, "finish" to write out the sliced files.

## Control Center
select printer, insert CMB, build job


# Using Optitrack
1. Insert USB key required to use Motive software
1. Launch Motive
1. Realign cameras, face/position cameras based on your volume of interest
    1. Need at least 4 camers, 5 is good trade-off, 6 is best (as long as the cameras do not interfere too much with each other)
    1. High as possible LED value, not too low value on EXP    1. Remove/hide as many markers as possible
    1. toggle between IR/RGB
1. Calibrate
1. Capture
    1. Vertical offset set to 0mm,
    1. Choose 3 markers to define ground plane
1. Use NatNet to broadcast position data to computer over network
