"""
INVERTED F ANTENNA DESIGN

This design models an inverted F antenna (IFA) on a dielectric substrate for a 868 MHz LoRa application.
Based on Texas Instruments Design Note DN023: https://www.ti.com/lit/an/swra228c/swra228c.pdf


"""

import emerge as em
import numpy as np
from emerge.plot import plot_ff, plot_ff_polar, plot_sp, smith


def main():

    
    # --- 1. Unit and simulation parameters --------------------------------------------------------------------------------------------------------------------------
    mm = 0.001  # meters per millimeter
    # Refined frequency range for antenna resonance around 868 MHz
    f1 = 670e6  # [Hz] start frequency
    f2 = 1070e6  # [Hz] stop frequency



        # --- 2. DN024 Antenna geometry dimensions (from reference table) --------------------------------------------------------------------------------------------
    # From Table 1 in DN024 (image 2):
    L1 = 9.0   # [mm] initial vertical section from feed
    L2 = 18.0  # [mm] shorting stub length
    L3 = 0.575   # [mm] horizontal meander sections
    L4 = 38.0  # [mm] total vertical extent of meander
    L5 = 1.0   # [mm] termination length
    L6 = None  # vertical segments - will be calculated
    
    W = 2.0    # [mm] trace width
    X1 = 63.0  # [mm] PCB width
    X2 = 25.0  # [mm] position reference
    Y = 43.0   # [mm] PCB length

    # # Calculate vertical segments between meanders
    # # Structure: L1 + 4*L6 + L5 should approximately equal L4
    # # L6 = (L4 - L1 - L5) / 4
    # L6 = (L4 - L1 - L5) / 4  # approximately 7.0 mm


    # If thickness is different than 0.8 mm of the design note, L6 should be adjusted
    th = 1.51  # [mm] substrate thickness
    pcb_margin = (6, 6, 6, 6 * 8)  # [mm] margin between traces and PCB edge (left, top, right, bottom)
    margin = 30 * mm  # [mm] margin around PCB for air box  (30)

    # --- 3. Create simulation object --------------------------------------------------------------------------------------------------------------------------
    model = em.Simulation("IFA", loglevel="INFO")

    # --- 4. Define PCB MIFA antenna geometry -----------------------------------------------------------------------------------------------------------------------
    pcbl = em.geo.PCB(thickness=th, unit=mm, material=em.lib.DIEL_FR4)

    # Create the feed line
    # Create the inverted F antenna
    ant = (
        pcbl.new(0, 0, W, (0, 1))  # Start at center, direction up
        .store("p1")  # Store feed point
        .straight(L1)  # First vertical section (L1)
        .turn(90, corner_type="square")  # Turn right
        .straight(L2 - W)  # First horizontal section (L3) - right
        .turn(-90, corner_type="square")  # Turn up
        .straight(L3)  # Vertical segment
        .turn(-90, corner_type="square")  # Turn left
        .straight(L4  - 2* W)  # Second horizontal section (L3) - left
        .turn(90, corner_type="square")  # Turn up
        .straight(L3)  # Vertical segment
        .turn(90, corner_type="square")  # Turn right
        .straight(L4 - 2 * W)  # Third horizontal section (L3) - right
        .turn(-90, corner_type="square")  # Turn up
        .straight(L3)  # Vertical segment
        .turn(-90, corner_type="square")  # Turn left
        .straight(L4 -  W )  # Fourth horizontal section (L3) - left
        )

    x0, y0, z0 = pcbl.origin

    # Compile the PCB antenna geometry
    ant_trace = pcbl.compile_paths(merge=True)
    # Add margins between traces and PCB bounding box
    pcbl.determine_bounds(*pcb_margin)

    # Create ground plane on bottom and top side of PCB
    ground_bottom = em.geo.XYPlate(
        pcbl.width * mm,  # type: ignore
        -pcb_margin[3] * mm - 0.5 * mm,  # type: ignore
        (x0 * mm - Y * mm / 2 - 7/2 * mm , y0 * mm + 1/2* mm, pcbl.z(1) * mm),
    ).set_material(em.lib.COPPER)



    # Generate the PCB dielectric geometry
    dielectric = pcbl.generate_pcb(merge=True)
    via = pcbl.generate_vias()

    # Generate air box around the PCB
    air = em.geo.Box(
        pcbl.width * mm + 2 * margin,  # type: ignore
        pcbl.length * mm + 2 * margin,  # type: ignore
        2 * margin,
        (
            x0 * mm - pcb_margin[0] * mm - margin  - Y * mm / 2 - 15/2 * mm,
            y0 * mm - margin - pcb_margin[3] * mm,
            z0 * mm - margin,
        ),
    )

    # Create port at the feed line start
    # modal_port = pcbl.modal_port(pcbl.load("p1"), 2, name="FeedPort")
    # modal_port_end = pcbl.modal_port(pcbl.load("p2"), 2, name="AntennaPort")
    lumped_port = pcbl.lumped_port(pcbl.load("p1"))
    # lumped_port_end = pcbl.lumped_port(pcbl.load("p2"), name="AntennaPort")

    # --- 5. Start mesh simulation  ----------------------------------------------------------------------------------------------------------------------------
    model.mw.set_resolution(0.4)   # 0.4 ou même 0.5 mm au début  (0.2)
    # Frequency sweep across the resonance
    model.mw.set_frequency_range(f1, f2, 11)

    # --- Combine geometry into simulation 
    model.commit_geometry()
    model.mesher.set_boundary_size(ant_trace, 2 * mm, growth_rate=5)

    # --- Generate mesh and preview 
    model.generate_mesh()
    model.view(selections=[lumped_port], plot_mesh=True, volume_mesh=False)  # type: ignore

    # --- Boundary conditions
    # Define modal port with specified orientation and impedance
    # port_bc = model.mw.bc.ModalPort(modal_port, 1)
    # port_bc_end = model.mw.bc.ModalPort(modal_port_end, 2)
    port_bc = model.mw.bc.LumpedPort(lumped_port, 1, Z0=50)
    # port_bc_end = model.mw.bc.LumpedPort(lumped_port_end, 2, Z0=50)
    model.mw.bc.AbsorbingBoundary(air.outside())

    # --- 6. Run frequency-domain solver and Smith Chart---------------------------------------------------------------------------------------------------------
    # model.view(plot_mesh=True, volume_mesh=False)
    data = model.mw.run_sweep(True, 1, multi_processing=True) # Un seul solveur tourne → 4× moins de mémoire utilisée

    # --- Post-process S-parameters 
    freqs = data.scalar.grid.freq
    freq_dense = data.scalar.grid.dense_f(1001)
    S11 = data.scalar.grid.model_S(1, 1, freq_dense)  # reflection coefficient
    plot_sp(freq_dense, [S11], labels=["S11"])  # plot loss in dB
    smith(S11, f=freq_dense, labels="S11")  # Smith chart of S11

    # --- 7.  Far-field radiation pattern ----------------------------------------------------------------------------------------------------------------------
    # Extract 2D cut at phi=0 plane and plot E-field magnitude
    ff1 = data.field.find(freq=868e6).farfield_2d((0, 0, 1), (1, 0, 0), air.outside())
    ff2 = data.field.find(freq=868e6).farfield_2d((0, 0, 1), (0, 1, 0), air.outside())

    plot_ff(
        ff1.ang * 180 / np.pi,  # type: ignore
        [ff1.normE / em.lib.EISO, ff2.normE / em.lib.EISO],
        dB=False,
        ylabel="Gain [dBi]",
    )  # linear plot vs theta
    plot_ff_polar(
        ff1.ang,  # type: ignore
        [ff1.normE / em.lib.EISO, ff2.normE / em.lib.EISO],
        dB=False,
        dBfloor=-20,
        title="Far-field polar plot of E-field magnitude vs angle",
    )  # polar plot of radiation

    # --- 8. 3D visualization ---------------------------------------------------------------------------------------------------------------------------------------
    model.display.add_objects(*model.all_geos())
    model.display.add_surf(
        *data.field.find(freq=868e6).cutplane(ds=1 * mm, z=-th / 2 * mm).scalar("Ey"),
        symmetrize=True,
        _fieldname="Ey",
    )
    model.display.show()

    # --- 9. 3D radiation visualization (OFFSET WRONG, TO BE FIXED) --------------------------------------------------------------------------------------------------
    ff3d = data.field.find(freq=868e6).farfield_3d(air.outside())
    surf = ff3d.surfplot(
        "normE",
        dB=False,
        rmax=40 * mm,
        offset=((pcbl.width * mm - 2 * pcb_margin[0] * mm) / 2, (32) / 2 * mm, 0),  # type: ignore
    )

    model.display.add_objects(*model.all_geos())
    model.display.add_surf(*surf, opacity=0.9, _fieldname="Far-Field |E|")
    model.display.show()


if __name__ == "__main__":
    main()
