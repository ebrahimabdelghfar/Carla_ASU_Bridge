import math
import numpy as np
import trajectory_planning_helpers.calc_vel_profile


def apply_apex_preview(vx_profile: np.ndarray,
                       el_lengths: np.ndarray,
                       kappa: np.ndarray,
                       d_preview: float = 0.0,
                       t_preview: float = 0.0,
                       ggv: np.ndarray = None,
                       drag_coeff: float = 0.0,
                       m_veh: float = 1.0,
                       dyn_model_exp: float = 1.0) -> np.ndarray:
    """
    Created by:
    Ebrahim Abdelghfar

    Documentation:
    Anticipates the braking phases in front of the apexes, i.e. the vehicle already holds apex speed some distance
    before the apex instead of still decelerating into it. A quasi-steady-state velocity profile (forward-backward
    solver) places the end of every braking phase exactly on the curvature maximum, which leaves no margin for
    steering actuator lag, path tracking error or grip overestimation, so the vehicle understeers out of the corner.

    The anticipation window of every apex is speed dependent, w = d_preview + t_preview * vx, because a fixed metric
    window is simultaneously too short at top speed and wastefully long at low apex speeds. It is limited to the
    section between the apex and its predecessor, therefore a slow corner can never bleed its speed back through a
    faster corner in front of it.

    Lowering single points can make the profile infeasible (the remaining braking distance is shorter than before), so
    a backward deceleration pass over the combined tire/drag limits is run afterwards if a ggv diagram is supplied.
    The result therefore stays inside the vehicle limits, it only brakes earlier.

    The track is assumed to be closed.

    Inputs:
    vx_profile:     velocity profile (unclosed, one entry per trajectory point)
    el_lengths:     element lengths between the trajectory points (closed, i.e. same number of entries as vx_profile)
    kappa:          curvature profile in rad/m (unclosed, same number of entries as vx_profile)
    d_preview:      [m] constant part of the anticipation window (0.0 for a purely speed proportional window)
    t_preview:      [s] speed proportional part of the anticipation window
    ggv:            ggv diagram [vx, ax_max, ay_max] used for the feasibility pass (None skips the pass)
    drag_coeff:     [m2*kg/m3] drag coefficient including all constants: 0.5 * c_w * A_front * rho_air
    m_veh:          [kg] vehicle mass
    dyn_model_exp:  [-] exponent of the combined acceleration model, range [1.0, 2.0]

    Outputs:
    vx_profile_mod: velocity profile with anticipated braking
    """

    if d_preview <= 0.0 and t_preview <= 0.0:
        return np.copy(vx_profile)

    no_points = vx_profile.size

    if el_lengths.size != no_points:
        raise RuntimeError("el_lengths must contain one entry per point of vx_profile (closed track)!")

    if kappa.size != no_points:
        raise RuntimeError("kappa must contain one entry per point of vx_profile!")

    # duplicate the lap so that every apex has its full window and its predecessor available without index wrapping
    s_double = np.insert(np.cumsum(np.tile(el_lengths, 2)), 0, 0.0)[:-1]
    vx_double = np.tile(vx_profile, 2)
    kappa_double = np.tile(kappa, 2)
    el_lengths_double = np.tile(el_lengths, 2)

    apex_inds = calc_apex_inds(vx_profile=vx_double, kappa=kappa_double)

    if apex_inds.size == 0:
        return np.copy(vx_profile)

    vx_mod = np.copy(vx_double)

    # ------------------------------------------------------------------------------------------------------------------
    # SHIFT THE APEX SPEEDS UPSTREAM -----------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    for j, ind_apex in enumerate(apex_inds):
        # the anticipation must not reach past the previous apex
        ind_limit = apex_inds[j - 1] if j > 0 else 0
        vx_apex = vx_double[ind_apex]

        i = ind_apex - 1

        while i > ind_limit and s_double[ind_apex] - s_double[i] <= d_preview + t_preview * vx_double[i]:
            if vx_mod[i] > vx_apex:
                vx_mod[i] = vx_apex
            i -= 1

    # ------------------------------------------------------------------------------------------------------------------
    # RESTORE FEASIBILITY (BACKWARD DECELERATION PASS) -----------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if ggv is not None:
        radii_double = np.abs(np.divide(1.0, kappa_double,
                                        out=np.full(kappa_double.size, np.inf),
                                        where=kappa_double != 0.0))

        for i in range(vx_mod.size - 2, -1, -1):
            ax_poss = trajectory_planning_helpers.calc_vel_profile.\
                calc_ax_poss(vx_start=vx_mod[i + 1],
                             radius=radii_double[i + 1],
                             ggv=ggv,
                             mu=1.0,
                             dyn_model_exp=dyn_model_exp,
                             drag_coeff=drag_coeff,
                             m_veh=m_veh,
                             mode='decel_backw')

            vx_poss = math.sqrt(math.pow(vx_mod[i + 1], 2) + 2 * ax_poss * el_lengths_double[i])

            # the limit was evaluated at point i + 1, it does not necessarily hold at point i where velocity and radius
            # differ, so re-evaluate it there and keep the stricter of the two (same refinement as the TPH solver)
            ax_poss = trajectory_planning_helpers.calc_vel_profile.\
                calc_ax_poss(vx_start=vx_poss,
                             radius=radii_double[i],
                             ggv=ggv,
                             mu=1.0,
                             dyn_model_exp=dyn_model_exp,
                             drag_coeff=drag_coeff,
                             m_veh=m_veh,
                             mode='decel_backw')

            vx_poss = min(vx_poss, math.sqrt(math.pow(vx_mod[i + 1], 2) + 2 * ax_poss * el_lengths_double[i]))

            if vx_poss < vx_mod[i]:
                vx_mod[i] = vx_poss

    # use the second lap, it carries the wrap-around influence of the first one
    return vx_mod[no_points:]


def calc_apex_inds(vx_profile: np.ndarray,
                   kappa: np.ndarray) -> np.ndarray:
    """
    Created by:
    Ebrahim Abdelghfar

    Documentation:
    Returns the apex indices of a velocity profile, i.e. the strict local minima. Plateaus of equal velocity are
    reduced to their point of highest curvature, which is the geometric apex the plateau belongs to.

    Inputs:
    vx_profile:     velocity profile
    kappa:          curvature profile in rad/m (same number of entries as vx_profile)

    Outputs:
    apex_inds:      indices of the apexes within vx_profile
    """

    apex_inds = []
    no_points = vx_profile.size
    i = 1

    while i < no_points - 1:
        if vx_profile[i] >= vx_profile[i - 1]:
            i += 1
            continue

        # velocity fell into a minimum, find the extent of a possible plateau
        ind_end = i
        while ind_end < no_points - 1 and vx_profile[ind_end + 1] == vx_profile[i]:
            ind_end += 1

        if ind_end < no_points - 1 and vx_profile[ind_end + 1] > vx_profile[i]:
            apex_inds.append(i + int(np.argmax(np.abs(kappa[i:ind_end + 1]))))

        i = ind_end + 1

    return np.array(apex_inds, dtype=np.int64)


# testing --------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    pass
