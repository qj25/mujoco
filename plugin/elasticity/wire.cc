#include <algorithm>
#include <cstddef>
#include <sstream>
#include <optional>
#include <iostream>
#include <unordered_set>
#include <set>
#include <chrono>

#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mujoco.h>
#include "wire.h"
#include "wire_utils.h"

namespace mujoco::plugin::elasticity {
namespace {

// Jet color palette for visualization
void scalar2rgba(float rgba[4], mjtNum stress[3], mjtNum vmin, mjtNum vmax) {
  // L2 norm of the stress
  mjtNum v = mju_norm3(stress);
  v = v < vmin ? vmin : v;
  v = v > vmax ? vmax : v;
  mjtNum dv = vmax - vmin;

  if (v < (vmin + 0.25 * dv)) {
    rgba[0] = 0;
    rgba[1] = 4 * (v - vmin) / dv;
    rgba[2] = 1;
  } else if (v < (vmin + 0.5 * dv)) {
    rgba[0] = 0;
    rgba[1] = 1;
    rgba[2] = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
  } else if (v < (vmin + 0.75 * dv)) {
    rgba[0] = 4 * (v - vmin - 0.5 * dv) / dv;
    rgba[1] = 1;
    rgba[2] = 0;
  } else {
    rgba[0] = 1;
    rgba[1] = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
    rgba[2] = 0;
  }
}

// Helper function to read numeric attributes
bool CheckAttr(const char* name, const mjModel* m, int instance) {
  char *end;
  std::string value = mj_getPluginConfig(m, instance, name);
  value.erase(std::remove_if(value.begin(), value.end(), isspace), value.end());
  strtod(value.c_str(), &end);
  return end == value.data() + value.size();
}

bool parseBoolOrDefault(const char* flag_str, bool default_value) {
  if (!flag_str || flag_str[0] == '\0') return default_value;

  std::string val = flag_str;
  std::transform(val.begin(), val.end(), val.begin(), ::tolower);  // case-insensitive

  if (val == "true" || val == "1") {
      return true;
  } else if (val == "false" || val == "0") {
      return false;
  } else {
      mju_warning("Invalid boolean value: '%s'. Using default (%s).", 
                  flag_str, default_value ? "true" : "false");
      return default_value;
  }
}

}  // namespace

// Factory function
std::optional<Wire> Wire::Create(const mjModel* m, mjData* d, int instance) {
  if (CheckAttr("twist", m, instance) && CheckAttr("bend", m, instance) && CheckAttr("twist_displace", m, instance)) {
    return Wire(m, d, instance);
  } else {
    mju_warning("Invalid parameter specification in wire plugin");
    return std::nullopt;
  }
}

// Plugin constructor
Wire::Wire(const mjModel* m, mjData* d, int instance) {
  // parameters were validated by the factor function
  mjtNum G = strtod(mj_getPluginConfig(m, instance, "twist"), nullptr);
  mjtNum E = strtod(mj_getPluginConfig(m, instance, "bend"), nullptr);
  vmax = strtod(mj_getPluginConfig(m, instance, "vmax"), nullptr);
  double overall_rot = strtod(mj_getPluginConfig(m, instance, "twist_displace"), nullptr);

  // Basic Settings:
  flat = parseBoolOrDefault(mj_getPluginConfig(m, instance, "flat"), true);
  pqsActive = parseBoolOrDefault(mj_getPluginConfig(m, instance, "pqsActive"), false);
  boolThetaOpt = parseBoolOrDefault(mj_getPluginConfig(m, instance, "boolThetaOpt"), false); // true for theta optimization with Newton's method (slower)
  boolIsoStr8 = parseBoolOrDefault(mj_getPluginConfig(m, instance, "boolIsoStr8"), true); // true if straight isotropic rod
  timing_enabled = parseBoolOrDefault(mj_getPluginConfig(m, instance, "timingEnabled"), false);
  calcEnergy = parseBoolOrDefault(mj_getPluginConfig(m, instance, "calcEnergy"), false);
  fullDyn = parseBoolOrDefault(mj_getPluginConfig(m, instance, "fullDyn"), false);
  
  // count plugin bodies
  n = 0;
  for (int i = 1; i < m->nbody; i++) {
    if (m->body_plugin[i] == instance) {
      if (!n++) {
        i0 = i;
      }
    }
  }
  n--;

  if (fullDyn) {
    pqsActive = false; // full dynamics implies PQS inactive
    new_bwi.clear();
    bodies_with_interactions.resize(n - 2);
    for (int i = 0; i < n - 2; i++) {
      bodies_with_interactions[i] = i + 1;
    }
  }

  // run forward kinematics to populate xquat (mjData not yet initialized)
  mju_zero(d->mocap_quat, 4*m->nmocap);
  mju_copy(d->qpos, m->qpos0, m->nq);
  mj_kinematics(m, d);
  // Initialize plugin_torque array
  plugin_torque.resize(m->nv);

  // Initialize DER variables
  nv = n - 1;
  bigL_bar = 0.;
  j_rot << 0., -1., 1., 0.; // 2D rotation CCW 90degrees

  // Initialize nodes and edges
  nodes.resize(nv + 2);
  edges.resize(nv + 1);
  // Clear qvel address and joint_nv
  qvel_addrs.clear();
  joint_nv.clear();

  // compute initial curvature and material properties
  for (int b = 0; b < n; b++) {
    int i = i0 + b;
    if (m->body_plugin[i] != instance) {
      mju_error("This body does not have the requested plugin instance");
    }

    // compute physical parameters
    int geom_i = m->body_geomadr[i];
    mjtNum J = 0, Iy = 0, Iz = 0;
    if (m->geom_type[geom_i] == mjGEOM_CYLINDER ||
        m->geom_type[geom_i] == mjGEOM_CAPSULE) {
      // https://en.wikipedia.org/wiki/Torsion_constant#Circle
      // https://en.wikipedia.org/wiki/List_of_second_moments_of_area
      J = mjPI * pow(m->geom_size[3*geom_i+0], 4) / 2;
      Iy = Iz = mjPI * pow(m->geom_size[3*geom_i+0], 4) / 4.;
    } else if (m->geom_type[geom_i] == mjGEOM_BOX) {
      // https://en.wikipedia.org/wiki/Torsion_constant#Rectangle
      // https://en.wikipedia.org/wiki/List_of_second_moments_of_area
      mjtNum h = m->geom_size[3*geom_i+1];
      mjtNum w = m->geom_size[3*geom_i+2];
      mjtNum a = std::max(h, w);
      mjtNum b = std::min(h, w);
      J = a*pow(b, 3)*(16./3.-3.36*b/a*(1-pow(b, 4)/pow(a, 4)/12));
      Iy = pow(2 * w, 3) * 2 * h / 12.;
      Iz = pow(2 * h, 3) * 2 * w / 12.;
    }
    // Set material properties
    edges[b].B << Iy * E, 0, 0, Iz * E;
    edges[b].beta = J * G;
    if (edges[b].B(0,0) != edges[b].B(1,1)) {boolIsoStr8 = false;}
    // init theta
    edges[b].theta = overall_rot/nv * b;
    edges[b].p_thetaloc = std::fmod(edges[b].theta, (2. * M_PI));    
    if (edges[b].p_thetaloc > M_PI) {
      edges[b].p_thetaloc -= 2 * M_PI;
    }
    edges[b].theta_displace = edges[b].p_thetaloc;

    // Store all joint velocity addresses for plugin bodies
    int joint_id = m->body_jntadr[i];
    if (joint_id >= 0) {  // Check if body has a joint
      qvel_addrs.push_back(m->jnt_dofadr[joint_id]);
      // Calculate number of DOFs for this joint
      int next_joint = joint_id + 1;
      int nv = (next_joint < m->njnt) ? 
               m->jnt_dofadr[next_joint] - m->jnt_dofadr[joint_id] : 
               m->nv - m->jnt_dofadr[joint_id];
      joint_nv.push_back(nv);
    }
  }

  updateVars(d);  // Call updateVars before InitBishopFrame
  initBishopFrame();
  transfBF();
  updateMatFrame();
  initO2M(d);
  initBaseOmega();
}

void Wire::updateVars(mjData* d) {
  // Update node positions and orientations
  for (int i = 0; i < nv + 2; i++) {
    int body_id = i0 + i;
    nodes[i].pos << d->xpos[3*body_id], d->xpos[3*body_id+1], d->xpos[3*body_id+2];
    // quats coeffs are in the order of x,y,z,w
    nodes[i].quat.coeffs() << d->xquat[4*body_id+1], d->xquat[4*body_id+2],
                            d->xquat[4*body_id+3], d->xquat[4*body_id];
    // std::cout << "pos"<<i<<"= " << nodes[i].pos << std::endl;
  }
  nodes[0].matframe = (nodes[0].quat * edges[0].qe_o2m_loc).toRotationMatrix();
  nodes[nv].matframe = (nodes[nv].quat * edges[nv].qe_o2m_loc).toRotationMatrix();

  bigL_bar = 0;
  // for i = 0
  edges[0].e = nodes[1].pos - nodes[0].pos;
  edges[0].e_bar = edges[0].e.norm();
  edges[0].torq.setZero();
  nodes[0].kb << 0.0, 0.0, 0.0;
  // nodes[nv+1].kb << 0.0, 0.0, 0.0;

  for (int i = 1; i < nv+1; i++) {
    edges[i].e = nodes[i+1].pos - nodes[i].pos;
    edges[i].e_bar = edges[i].e.norm();
    edges[i].l_bar = edges[i].e_bar + edges[i-1].e_bar;
    bigL_bar += edges[i].l_bar;
    edges[i].torq.setZero();
    // Calculate curvature
    // std::cout << "e0: " << edges[i-1].e << std::endl;
    // std::cout << "e1: " << edges[i].e << std::endl;
    nodes[i].phi_i = WireUtils::calculateAngleBetween(edges[i-1].e, edges[i].e);
    // std::cout << "re0: " << edges[i-1].e << std::endl;
    // std::cout << "re1: " << edges[i].e << std::endl;

    nodes[i].k = 2. * std::tan(nodes[i].phi_i / 2.);
    nodes[i].kb = 2. * edges[i-1].e.cross(edges[i].e) / 
                  (edges[i-1].e_bar * edges[i].e_bar + edges[i-1].e.dot(edges[i].e));
    // std::cout << "kb"<<i<<": " << nodes[i].kb << std::endl;
    // std::cout << "e"<<i<<": " << edges[i].e << std::endl;
  }
  bigL_bar /= 2.0;
}

void Wire::Compute(const mjModel* m, mjData* d, int instance) {
  using namespace std::chrono;
  high_resolution_clock::time_point start, end;
  if (timing_enabled) start = high_resolution_clock::now();
  updateVars(d);  // Call updateVars at the start of Compute
  // Update bishop frame
  updateBishopFrame(d);

  // Zero plugin_torque array
  mju_zero(plugin_torque.data(), m->nv);

  // Calculate torques
  if (fullDyn) {
    // For full dynamics, we assume all bodies are interacting
    for (size_t i = 0; i < bodies_with_interactions.size(); i++) {
      int bwi = bodies_with_interactions[i];        
      // update interacted theta
      updateTheta(get_thetaloc(d,bwi),bwi);
    }
  }
  else if (pqsActive) {
    // For PQSCenterlineTwist
    // do interaction checking
    detectInteractions(m, d);
    // Check if there are any bodies with interactions
    // if (false) {
    if (bodies_with_interactions.empty()) {
      updateTheta(get_thetaloc(d,nv),nv);
      splitTheta(0, nv);
    }
    else {
      // carry out o2m init and theta_displace for newbwi
      for (int nb_it : new_bwi) {
        updateNewBWI(d, nb_it);
      }
      // interate through full bwi
      for (size_t i = 0; i < bodies_with_interactions.size(); i++) {
        int bwi = bodies_with_interactions[i];        
        // update interacted theta
        updateTheta(get_thetaloc(d,bwi),bwi);
        // first bwi split with edge[0]
        if (i == 0) {
          splitTheta(0, bwi);
        }
        else {
          int prev_bodyid = bodies_with_interactions[i-1];
          splitTheta(prev_bodyid, bwi);
        }
        // last bwi split with edge[nv]
        if (i == bodies_with_interactions.size() - 1) {
          updateTheta(get_thetaloc(d,nv),nv);
          splitTheta(bwi, nv);
        }
      }
    }
  }
  // For QSCenterlineTwist
  else {
    updateTheta(get_thetaloc(d,nv),nv);
    splitTheta(0, nv);
  }

  updateMatFrame();

  // update material curvature
  for (int i = 1; i < nv + 1; i++) {
    for (int j = i-1; j < i+1; j++) {
      nodes[j].omega.col(i-j) << nodes[i].kb.dot(nodes[j].matframe.col(2)),
          - nodes[i].kb.dot(nodes[j].matframe.col(1));
    }
    // std::cout << "bf: " << edges[i].bf << std::endl;
    // std::cout << "matframe: " << nodes[i].matframe << std::endl;
    // std::cout << "omega: " << nodes[i].omega << std::endl;
  }

  // Calculate twist using dE/dtheta |========================================
  // =========================================================================
  // bend contribution
  for (int j = 0; j < nv + 1; j++) {
    if (j > 0) {
      edges[j].torq(0) -= (
        nodes[j].omega.col(0).transpose().dot(
          j_rot * edges[j].B * (
            nodes[j].omega.col(0) - nodes[j].omegaBase.col(0) //insert omega base j,j
          )
        )
      )/edges[j].l_bar;
    }
    if (j < nv) {
      edges[j].torq(0) -= (
        nodes[j].omega.col(1).transpose().dot(
          j_rot * edges[j].B * (
            nodes[j].omega.col(1) - nodes[j].omegaBase.col(1) //insert omega base j,j+1
          )
        )
      )/edges[j+1].l_bar;
    }
  }
  // twist contribution |=====================================================
  // For QSCenterlineTwist
  edges[nv].torq(0) -= 2.0 * (
    edges[nv].beta * (edges[nv].theta - edges[nv-1].theta) / edges[nv].l_bar
  );
  edges[0].torq(0) -= 2.0 * (
    - edges[1].beta * (edges[1].theta - edges[0].theta) / edges[1].l_bar
  );
  // Additional for PQSCenterlineTwist
  if (pqsActive || fullDyn) {
    if (!bodies_with_interactions.empty()) {
      for (int bwi : bodies_with_interactions) {
        // 
        edges[bwi].torq(0) -= 2.0 * (
          edges[bwi].beta * (edges[bwi].theta - edges[bwi-1].theta) / edges[bwi].l_bar
        );
        edges[bwi].torq(0) -= 2.0 * (
          - edges[bwi+1].beta * (edges[bwi+1].theta - edges[bwi].theta) / edges[bwi+1].l_bar
        );
      }
    }
  }
  
  // Calculate bend using dE/dgamma |=========================================
  // =========================================================================
  // bend contribution
  for (int j = 0; j < nv + 1; j++) {
    for (int k = 0; k < 2; k++) {
      // k reps material axis
      // calculate dkb/dthetak, then domega/dthetak, then dEbend/dthetak
      // restoring tau = -dEbend/dthetak
      Eigen::Vector3d c_temp = nodes[j].matframe.col(k+1).cross(edges[j].e);
      // dkb0
      Eigen::Vector2d domega;
      Eigen::Vector3d dkb;
      if (j > 0) {
        dkb = (
          2.0*edges[j-1].e.cross(c_temp)
          - (edges[j-1].e.dot(c_temp))*nodes[j].kb
        ) / (edges[j-1].e_bar * edges[j].e_bar + edges[j-1].e.dot(edges[j].e));
        // domega^j-1_j
        domega << dkb.dot(nodes[j-1].matframe.col(2)),
                - dkb.dot(nodes[j-1].matframe.col(1));
        // if (j==nv) {
        //   std::cout << "k: " << k << std::endl;
        //   std::cout << "c_temp: " << c_temp << std::endl;
        //   // std::cout << "matframe: " << nodes[j].matframe.col(k+1) << std::endl;
        //   // std::cout << "e: " << edges[j].e << std::endl;
        //   std::cout << "dkb: " << dkb << std::endl;
        //   std::cout << "domegaj-1_j: " << domega << std::endl;
        // }
        // std::cout << "kb"<<j<<": " << nodes[j].kb << std::endl;
        // std::cout << "domegaA1: " << domega << std::endl;

        edges[j].torq(k+1) -= (
          domega.transpose().dot(
            edges[j-1].B * (
              nodes[j-1].omega.col(1) - nodes[j-1].omegaBase.col(1) //insert omega base j-1,j
            )
          )
        )/edges[j].l_bar;
        // std::cout << "torqA1"<<j<<": " << edges[j].torq << std::endl;
        // domega^j_j
        domega << dkb.dot(nodes[j].matframe.col(2)),
                - dkb.dot(nodes[j].matframe.col(1));
        edges[j].torq(k+1) -= (
          domega.transpose().dot(
            edges[j].B * (
              nodes[j].omega.col(0) - nodes[j].omegaBase.col(0) //insert omega base j,j
            )
          )
        )/edges[j].l_bar;
        // std::cout << "domegaA2: " << domega << std::endl;
        // std::cout << "torqA2"<<j<<": " << edges[j].torq << std::endl;
      }
      // std::cout << "j: " << j << std::endl;
      // std::cout << "kb: " << nodes[j].kb << std::endl;
      // std::cout << "ctemp: " << c_temp << std::endl;
      // std::cout << "omega: " << nodes[j].omega << std::endl;

      if (j < nv) {
        // dkb1
        dkb = (
          - 2.0*edges[j+1].e.cross(c_temp)
          - (edges[j+1].e.dot(c_temp))*nodes[j+1].kb
        ) / (edges[j+1].e_bar * edges[j].e_bar + edges[j+1].e.dot(edges[j].e));
        // domega^j_j+1
        domega << dkb.dot(nodes[j].matframe.col(2)),
                - dkb.dot(nodes[j].matframe.col(1));
        // std::cout << "dotprod1: " << dkb.dot(nodes[j].matframe.col(2)) << std::endl;
        // std::cout << "dotprod2: " << -dkb.dot(nodes[j].matframe.col(1)) << std::endl;
        // std::cout << "domegaj_j+1: " << domega << std::endl;
        // if (j==0) {
        //   std::cout << "k: " << k << std::endl;
        //   std::cout << "c_temp: " << c_temp << std::endl;
        //   // std::cout << "matframe: " << nodes[j].matframe.col(k+1) << std::endl;
        //   // std::cout << "e: " << edges[j].e << std::endl;
        //   std::cout << "dkb: " << dkb << std::endl;
        //   std::cout << "matframe: " << nodes[j].matframe << std::endl;
        //   std::cout << "dotcol2: " << dkb.dot(nodes[j].matframe.col(2)) << std::endl;
        //   std::cout << "dotcol1: " << -dkb.dot(nodes[j].matframe.col(1)) << std::endl;
        //   std::cout << "domegaj_j+1: " << domega << std::endl;
        // }
        // std::cout << "kb"<<j+1<<": " << nodes[j+1].kb << std::endl;
        // std::cout << "domegaB1: " << domega << std::endl;

        edges[j].torq(k+1) -= (
          domega.transpose().dot(
            edges[j].B * (
              nodes[j].omega.col(1) - nodes[j].omegaBase.col(1) //insert omega base j,j+1
            )
          )
        )/edges[j+1].l_bar;
        // std::cout << "torqB1"<<j<<": " << edges[j].torq << std::endl;
        // domega^j+1_j+1
        domega << dkb.dot(nodes[j+1].matframe.col(2)),
                - dkb.dot(nodes[j+1].matframe.col(1));
        edges[j].torq(k+1) -= (
          domega.transpose().dot(
            edges[j+1].B * (
              nodes[j+1].omega.col(0) - nodes[j+1].omegaBase.col(0) //insert omega base j+1,j+1
            )
          )
        )/edges[j+1].l_bar;
        // std::cout << "domegaB2: " << domega << std::endl;
        // std::cout << "torqB2"<<j<<": " << edges[j].torq << std::endl;
      }

      // twist contribution |=================================================
      // For PQSCenterlineTwist
      if (pqsActive || fullDyn) {
        // Sum of torques from bodies with interactions > j
        double sum_torq_gt_j = 0.0;
        // Sum of torques from bodies with interactions > j-1
        double sum_torq_gt_jm1 = 0.0;
        
        // Calculate sums - bodies_with_interactions is sorted in increasing order
        for (int i = bodies_with_interactions.size() - 1; i >= 0; i--) {
          int bwi = bodies_with_interactions[i];
          if (bwi < j) {
            break;  // Since sorted, all remaining values will be <= j
          }
          sum_torq_gt_jm1 += edges[bwi].torq(0);
          if (bwi > j) {
            sum_torq_gt_j += edges[bwi].torq(0);
          }
        }
        sum_torq_gt_jm1 += edges[nv].torq(0);
        sum_torq_gt_j += edges[nv].torq(0);

        // dphi_j/dgammaj
        if (j > 0) {
          edges[j].torq(k+1) += (
            sum_torq_gt_jm1 * (
              nodes[j].kb
            ).dot(c_temp)
          )/(2.0*edges[j].e_bar);
        }
        // dphi_j+1/dgammaj
        if (j < nv) {
          edges[j].torq(k+1) += (
            sum_torq_gt_j * (
              nodes[j+1].kb
            ).dot(c_temp)
          )/(2.0*edges[j].e_bar);
        }
      }
      // For QSCenterlineTwist
      else {
        // dphi_j/dgammaj
        if (j > 0) {
          edges[j].torq(k+1) += (
            edges[nv].torq(0) * (
              nodes[j].kb
            ).dot(c_temp)
          )/(2.0*edges[j].e_bar);
        }
        // dphi_j+1/dgammaj
        if (j < nv) {
          edges[j].torq(k+1) += (
            edges[nv].torq(0) * (
              nodes[j+1].kb
            ).dot(c_temp)
          )/(2.0*edges[j].e_bar);
        }
      }
      // std::cout << "torq"<<j<<": " << edges[j].torq(0) << std::endl;
      // std::cout << "torq"<<j<<": " << edges[j].torq << std::endl;
      
      // HARD
      // for dyn material centerline twist, remember to do for i = 0
    }
  }
  // for (int j = 0; j < nv + 1; j++) {
    // std::cout << "piece " << j << " twist= " << edges[j].torq(0) << std::endl;
  // }

  // =========================================================================
  // =========================================================================
  // for (int i = 0; i < nv+1; i++) {
    // std::cout << "torq"<<i<<": " << edges[i].torq << std::endl;
  // }


  // // Apply torques to MuJoCo bodies
  // convert to global coordinates and apply torque to com
  double applyFT_elapsed_ms = 0.0;
  for (int b = 0; b < n; b++)  {
    int i = i0 + b;
    mjtNum lfrc[3] = {edges[b].torq(0), edges[b].torq(1), edges[b].torq(2)};
    mjtNum xfrc[3] = {0};
    Eigen::Quaterniond q_body(nodes[b].matframe);
    mjtNum material_quat[4] = {q_body.w(), q_body.x(), q_body.y(), q_body.z()};
    // std::cout << "material_quat: " << material_quat[0] << material_quat[1] << material_quat[2] << material_quat[3] << std::endl;
    // std::cout << "real_quat: " << d->xquat[4*i] << d->xquat[4*i+1] << d->xquat[4*i+2] << d->xquat[4*i+3] << std::endl;
    // std::cout << "lfrc"<<b<<": " << lfrc[0]<< ", " << lfrc[1]<< ", "  << lfrc[2]<< ", "  << std::endl;
    // std::cout << "n_pieces"<< b << "===================" << std::endl;
    // for (int jj = 1; jj < nv+2; jj++) {
    //   std::cout << "torq"<<jj<<": " << plugin_torque.data()[jj*3] << " "<<plugin_torque.data()[jj*3+1]<< " " <<plugin_torque.data()[jj*3+2] << std::endl;
    //   // std::cout << "torq"<<i<<": " << plugin_torque.data()[0] << std::endl;
    // }
    // std::cout << d->qfrc_passive << std::endl;
    mju_rotVecQuat(xfrc, lfrc, material_quat);
    if (timing_enabled) {
      high_resolution_clock::time_point t0 = high_resolution_clock::now();
      mj_applyFT(m, d, 0, xfrc, d->xpos+3*i, i, plugin_torque.data());
      high_resolution_clock::time_point t1 = high_resolution_clock::now();
      applyFT_elapsed_ms += duration<double, std::milli>(t1 - t0).count();
    } else {
      mj_applyFT(m, d, 0, xfrc, d->xpos+3*i, i, plugin_torque.data());
    }
  }
  if (timing_enabled) {
    total_applyFT_time_ms += applyFT_elapsed_ms;
  }

  // Add our forces to qfrc_passive
  mju_addTo(d->qfrc_passive, plugin_torque.data(), m->nv);

  // calculate Energy if requested
  if (calcEnergy) {calculateEnergy();}

  // Store plugin_torque in plugin state
  mjtNum* plugin_state = d->plugin_state + m->plugin_stateadr[instance];
  mju_copy(plugin_state, plugin_torque.data(), m->nv);
  plugin_state[m->nv] = E_total;

  if (timing_enabled) {
    end = high_resolution_clock::now();
    double elapsed = duration<double, std::milli>(end - start).count();
    total_compute_time_ms += elapsed;
    compute_call_count++;
  }
}

void Wire::calculateEnergy() {
  double E_bend;
  double E_twist;
  E_bend = 0.0;
  E_twist = 0.0;
  E_total = 0.0;

  for (int i = 1; i < nv+1; i++) {
    for (int j = i-1; j < i+1; j++) {
      E_bend += (
        (nodes[j].omega.col(i-j)-nodes[j].omegaBase.col(i-j)).transpose().dot(
          edges[j].B * (
            nodes[j].omega.col(i-j) - nodes[j].omegaBase.col(i-j) //insert omega base j,j
          )
        )
      )/(2.0*edges[i].l_bar);

      E_twist += (
        edges[i].beta 
        * (edges[i].theta - edges[i-1].theta) * (edges[i].theta - edges[i-1].theta)
        / edges[i].l_bar
      );
    }
  }
  E_total = E_bend + E_twist;
}

void Wire::Visualize(const mjModel* m, mjData* d, mjvScene* scn, int instance) {
  if (!vmax) {
    return;
  }

  for (int b = 0; b < n; b++) {
    int i = i0 + b;
    // set geometry color based on stress norm
    mjtNum stress_m[3] = {0};
    scalar2rgba(m->geom_rgba + 4*m->body_geomadr[i], stress_m, 0, vmax);
  }
}

void Wire::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.elasticity.wire";
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  const char* attributes[] = {
    "twist", "bend", "flat",
    "vmax", "twist_displace",
    "pqsActive", "boolThetaOpt",
    "boolIsoStr8", "timingenabled",
    "calcEnergy", "fullDyn"
  };
  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;
  
  // Allocate state space for storing qfrc_passive
  // plugin.nstate = +[](const mjModel* m, int instance) { return 0; };
  plugin.nstate = +[](const mjModel* m, int instance) { 
    return m->nv + 1;  // Size of qfrc_passive + E_total
  };

  plugin.init = +[](const mjModel* m, mjData* d, int instance) {
    auto elasticity_or_null = Wire::Create(m, d, instance);
    if (!elasticity_or_null.has_value()) {
      return -1;
    }
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(
        new Wire(std::move(*elasticity_or_null)));
    return 0;
  };

  plugin.destroy = +[](mjData* d, int instance) {
    auto* elasticity = reinterpret_cast<Wire*>(d->plugin_data[instance]);
    elasticity->PrintComputeTiming();
    delete elasticity;
    d->plugin_data[instance] = 0;
  };

  plugin.compute = +[](const mjModel* m, mjData* d, int instance, int capability_bit) {
    auto* elasticity = reinterpret_cast<Wire*>(d->plugin_data[instance]);
    elasticity->Compute(m, d, instance);
  };

  plugin.visualize = +[](const mjModel* m, mjData* d, const mjvOption* opt, mjvScene* scn,
                         int instance) {
    auto* elasticity = reinterpret_cast<Wire*>(d->plugin_data[instance]);
    elasticity->Visualize(m, d, scn, instance);
  };

  mjp_registerPlugin(&plugin);
}

void Wire::initBaseOmega() {
  if (!flat) {
    // initialize base material curvature (omega)
    for (int i = 1; i < nv+1; i++) {
      for (int j = i-1; j < i+1; j++) {
        nodes[j].omegaBase.col(i-j) << nodes[i].kb.dot(nodes[j].matframe.col(2)),
            - nodes[i].kb.dot(nodes[j].matframe.col(1));
        // Set boolIsoStr8 to false if omegaBase.col(i-j) is not zero
        if (nodes[j].omegaBase.col(i-j).norm() > 1e-8) {
          boolIsoStr8 = false;
      }

      }
    }
  }
  else {
    for (int i = 0; i < nv+1; i++) {
      nodes[i].omegaBase.setZero();
    }
  }
}

void Wire::initBishopFrame() {
  const double parll_tol = 1e-6;
  Eigen::Matrix3d bf0_bar;
  
  // Initialize first column of bishop frame
  bf0_bar.col(0) = edges[0].e / edges[0].e_bar;
  
  // Initialize second column with cross product with z-axis
  bf0_bar.col(1) = bf0_bar.col(0).cross(Eigen::Vector3d(0, 0, 1));
  
  // If the cross product is too small (vectors are nearly parallel), use y-axis instead
  if (bf0_bar.col(1).norm() < parll_tol) {
    bf0_bar.col(1) = bf0_bar.col(0).cross(Eigen::Vector3d(0, 1, 0));
  }
  
  // Normalize the second column
  bf0_bar.col(1) /= bf0_bar.col(1).norm();
  
  // Initialize third column as cross product of first two columns
  bf0_bar.col(2) = bf0_bar.col(0).cross(bf0_bar.col(1));
  edges[0].bf = bf0_bar;
  // std::cout << "bf000000: " << edges[0].bf << std::endl;
  // std::cout << "e00000000: " << edges[0].e << std::endl;
  // std::cout << "======================================" << std::endl;
}

void Wire::initO2M(mjData* d) {
  // possible to remove o2m_loc? saves computation
  // but will there be lose in generality?
  for (int i = 0; i < nv + 1; i++) {
    Eigen::Quaterniond q_o = nodes[i].quat;
    // Convert bishop frame to quaternion
    Eigen::Quaterniond q_b(nodes[i].matframe);
    q_b.normalize();
  
    // Calculate quaternion error
    Eigen::Quaterniond q_error = q_b * q_o.inverse();
    q_error.normalize();
    
    // Transform error into local frame of q_o
    edges[i].qe_o2m_loc = q_o.inverse() * q_error * q_o;
    edges[i].qe_o2m_loc.normalize();
    // if (i==0) {
    //   std::cout << "qe_o2m_loc0: " << edges[0].qe_o2m_loc << std::endl;
    //   std::cout << "q_o: " << q_o << std::endl;
    //   std::cout << "q_b: " << q_b << std::endl;
    //   std::cout << "q_error: " << q_error << std::endl;
    //   std::cout << "actualbf0: " << edges[0].bf << std::endl;
    //   std::cout << "bf0again: " << (nodes[0].quat * edges[0].qe_o2m_loc).toRotationMatrix() << std::endl;
    // }
  }
}

void Wire::updateO2M(mjData* d, int idx) {
  Eigen::Quaterniond q_o = nodes[idx].quat;
  // Convert bishop frame to quaternion
  Eigen::Quaterniond q_b(edges[idx].bf);
  q_b.normalize();

  // Calculate quaternion error
  Eigen::Quaterniond q_error = q_b * q_o.inverse();
  q_error.normalize();
  
  // Transform error into local frame of q_o
  edges[idx].qe_o2m_loc = q_o.inverse() * q_error * q_o;
  edges[idx].qe_o2m_loc.normalize();
}

void Wire::updateBishopFrame(mjData* d) {
  // Transform to local and convert to matrix (no need to transpose)
  edges[0].bf = (nodes[0].quat * edges[0].qe_o2m_loc).toRotationMatrix();
  // std::cout << "bf0: " << edges[0].bf << std::endl;
  // std::cout << "======================================" << std::endl;
  // Transfer bishop frame along the wire
  transfBF();
}

bool Wire::transfBF() {
  bool bf_align = true;
  // for (int i = 0; i < nv + 2; i++) {
    // std::cout << "x" << i << ": " << nodes[i].pos << std::endl;
  // }
  // std::cout << "bf0: " << edges[0].bf << std::endl;

  for (int i = 1; i < nv+1; i++) {
    edges[i].bf.col(0) = edges[i].e / edges[i].e_bar;
    if (nodes[i].kb.norm() == 0) {
      edges[i].bf.col(1) = edges[i-1].bf.col(1);
    } else {
      edges[i].bf.col(1) = WireUtils::rotateVector3(
        edges[i-1].bf.col(1),
        nodes[i].kb / nodes[i].kb.norm(),
        nodes[i].phi_i
      );
      if (std::abs(edges[i].bf.col(1).dot(edges[i].bf.col(0))) > 1e-1) {
        bf_align = false;
      }
    }
    edges[i].bf.col(2) = edges[i].bf.col(0).cross(edges[i].bf.col(1));
    // std::cout << "e"<< i <<": " << edges[i].e << std::endl;
    // std::cout << "bf"<< i <<": " << edges[i].bf << std::endl;
  }
  return bf_align;
}

double Wire::get_thetaloc(mjData* d, int idx) {
  // Gets local theta rotation (overall_rot excluded)
  // Get the bishop frame at the end (no need to transpose)
  Eigen::Matrix3d mat_bn = edges[idx].bf;

  // Calculate the orientation in the material frame
  Eigen::Matrix3d mat_mn = (nodes[idx].quat * edges[idx].qe_o2m_loc).toRotationMatrix();

  // Calculate the angle between the second columns of mat_bn and mat_mn around mat_bn's first column
  Eigen::Vector3d v1 = mat_bn.col(1);
  Eigen::Vector3d v2 = mat_mn.col(1);
  Eigen::Vector3d va = mat_bn.col(0);

  double theta_diff = (
    WireUtils::calculateAngleBetween2b(v1,v2,va)
    + edges[idx].theta_displace
  );
  if (theta_diff > M_PI) {
    theta_diff -= 2 * M_PI;
  }
  return theta_diff;
}

void Wire::updateTheta(double theta_loc, int idx) {
  double diff_theta = theta_loc - edges[idx].p_thetaloc;

  // Account for 2pi rotation
  if (std::abs(diff_theta) < M_PI) {
    edges[idx].theta += diff_theta;
  } else if (diff_theta > 0.) {
    edges[idx].theta += diff_theta - (2 * M_PI);
  } else {
    edges[idx].theta += diff_theta + (2 * M_PI);
  }
  edges[idx].p_thetaloc = theta_loc;
}

void Wire::splitTheta(int startId, int endId) {
  int n_edges = endId - startId + 1;
  if (boolIsoStr8 || !boolThetaOpt) {
    // evenly splits theta from startId to endId
    double d_theta;
    d_theta = (edges[endId].theta - edges[startId].theta) / (n_edges-1);
    for (int i = 0; i < (n_edges); i++) {
      edges[i+startId].theta = d_theta * i + edges[startId].theta;
    }
  }
  else {
    // use latest theta for initial guess
    Eigen::VectorXd theta_opt(n_edges);
    for (int i = 0; i < (n_edges); i++) {
      theta_opt[i] = edges[i+startId].theta;
    }
    newton_minimize(theta_opt, startId, endId);
    // reassign back to class var
    for (int i = 0; i < (n_edges); i++) {
      edges[i+startId].theta = theta_opt[i];
    }
  }
}

double Wire::compute_energy(const Eigen::VectorXd& theta, int startId, int endId) {
  double E_total;
  updateSingleMatFrame(theta(0), startId);
  nodes[startId].omega.col(1) << nodes[startId+1].kb.dot(nodes[startId].matframe.col(2)),
      - nodes[startId+1].kb.dot(nodes[startId].matframe.col(1));
  for (int i=startId+1; i<endId+1; i++) {
    updateSingleMatFrame(theta(i-startId), i);
    //update omega
    nodes[i].omega.col(0) << nodes[i].kb.dot(nodes[i].matframe.col(2)),
      - nodes[i].kb.dot(nodes[i].matframe.col(1));
    nodes[i].omega.col(1) << nodes[i+1].kb.dot(nodes[i].matframe.col(2)),
      - nodes[i+1].kb.dot(nodes[i].matframe.col(1));
    // Calc E
    for (int j=i-1; j<i+1; j++) {
      E_total += (
        (nodes[j].omega.col(i-j) - nodes[j].omegaBase.col(i-j)).transpose().dot(
          edges[j].B * (
            nodes[j].omega.col(i-j) - nodes[j].omegaBase.col(i-j)
          )
        )
      )/(2*edges[i].l_bar);
    }
    E_total += edges[i].beta*(theta(i-startId)-theta(i-startId-1))/edges[i].l_bar;
  }
  
  return E_total;
}

Eigen::VectorXd Wire::compute_gradient(const Eigen::VectorXd& theta, int startId, int endId) {
  // Create a vector of size n (number of edges) and zero it
  int n_edges = endId-startId+1;
  Eigen::VectorXd dEdtheta(n_edges);
  dEdtheta.setZero();

  // Calculate gradient
  // exclude startId and endId because they are constants
  // -(physically held fixed)-
  for (int j = startId+1; j < endId; j++) {
    // update matframe
    updateSingleMatFrame(theta(j-startId), j);
    // update omega
    for (int i = j; i < j+2; i++) {
      nodes[j].omega.col(i-j) << nodes[i].kb.dot(nodes[j].matframe.col(2)),
          - nodes[i].kb.dot(nodes[j].matframe.col(1));
    }
    if (j > 0) {
      // Bend contribution
      dEdtheta(j-startId) += (
        nodes[j].omega.col(0).transpose().dot(
          j_rot * edges[j].B * (
            nodes[j].omega.col(0) - nodes[j].omegaBase.col(0)
          )
        )
      )/edges[j].l_bar;
      // Twist contribution
      dEdtheta(j-startId) += 2.0 * (
        edges[j].beta * (theta(j-startId) - theta(j-startId-1)) / edges[j].l_bar
      );
    }
    if (j < nv) {
      // Bend contribution
      dEdtheta(j-startId) += (
        nodes[j].omega.col(1).transpose().dot(
          j_rot * edges[j].B * (
            nodes[j].omega.col(1) - nodes[j].omegaBase.col(1)
          )
        )
      )/edges[j+1].l_bar;
      // Twist contribution
      dEdtheta(j-startId) += 2.0 * (
        - edges[j+1].beta * (theta(j-startId+1) - theta(j-startId)) / edges[j+1].l_bar
      );
    }
  }

  return dEdtheta;
}

void Wire::compute_tridiagonal_hessian(const Eigen::VectorXd& theta,
                                  int startId, int endId,
                                  std::vector<double>& lower,  // a: below diagonal (n-1)
                                  std::vector<double>& diag,   // b: diagonal (n)
                                  std::vector<double>& upper)  // c: above diagonal (n-1)
{
  /*
    - Ensure 'compute_gradient' is called first 
      so that matframe and omega are updated
    - Enforcing constraint of fixed variable:
      set H(i,i) = 1.0 and H(i,j) = H(j,i) = 0.0 for theta_i fixed,
      in this case, i = 0 and i = -1.
  */

  // upper_i is H(i,i+1); lower_i is H(i+1,i)
  int n = endId - startId + 1;
  diag.resize(n, 1.0);
  lower.resize(n - 1, 0.0);
  upper.resize(n - 1, 0.0);
  for (int i=startId+1; i<endId; i++) {
    if (i > startId+1) {
      lower[i-1-startId] = -2*edges[i].beta/edges[i].l_bar;
      diag[i-startId] -= lower[i-1-startId];
      diag[i-startId] += (
        nodes[i].omega.col(0).transpose().dot(
          j_rot.transpose() * edges[i].B * (
            j_rot * nodes[i].omega.col(0)
          )
        )
      )/edges[i].l_bar;
      diag[i-startId] += -(
        nodes[i].omega.col(0).transpose().dot(
          edges[i].B * (
            nodes[i].omega.col(0) - nodes[i].omegaBase.col(0)
          )
        )
      )/edges[i].l_bar;
    }
    if (i < endId-1) {
      upper[i-startId] = -2*edges[i+1].beta/edges[i+1].l_bar;
      diag[i-startId] -= upper[i-startId];
      diag[i-startId] += (
        nodes[i].omega.col(1).transpose().dot(
          j_rot.transpose() * edges[i].B * (
            j_rot * nodes[i].omega.col(1)
          )
        )
      )/edges[i+1].l_bar;
      diag[i-startId] += -(
        nodes[i].omega.col(1).transpose().dot(
          edges[i].B * (
            nodes[i].omega.col(1) - nodes[i].omegaBase.col(1)
          )
        )
      )/edges[i+1].l_bar;
    }
  }
}

// Thomas algorithm: solves Ax = d where A is tridiagonal
void Wire::solve_tridiagonal(const std::vector<double>& lower,
                      const std::vector<double>& diag,
                      const std::vector<double>& upper,
                      const Eigen::VectorXd& rhs,
                      Eigen::VectorXd& x)
{
  int n = diag.size();
  std::vector<double> c_prime(n - 1);
  std::vector<double> d_prime(n);
  x.resize(n);

  // Forward sweep
  c_prime[0] = upper[0] / diag[0];
  d_prime[0] = rhs[0] / diag[0];
  for (int i = 1; i < n; ++i) {
    double denom = diag[i] - lower[i - 1] * c_prime[i - 1];
    c_prime[i < n - 1 ? i : 0] = (i < n - 1) ? upper[i] / denom : 0.0;
    d_prime[i] = (rhs[i] - lower[i - 1] * d_prime[i - 1]) / denom;
  }

  // Back substitution
  x[n - 1] = d_prime[n - 1];
  for (int i = n - 2; i >= 0; --i)
    x[i] = d_prime[i] - c_prime[i] * x[i + 1];
}

// Backtracking line search
double Wire::line_search(const Eigen::VectorXd& theta,
                  int startId, int endId,
                  const Eigen::VectorXd& direction,
                  const Eigen::VectorXd& grad,
                  double alpha,
                  double rho,
                  double c)
{
  // removed Armijo condition as it gave non-optimized theta
  double E0 = compute_energy(theta, startId, endId);
  // double dphi0 = grad.dot(direction);

  Eigen::VectorXd trial;
  while (alpha > 1e-8) {
    trial = theta + alpha * direction;
    double E_trial = compute_energy(trial, startId, endId);
    // if (E_trial <= E0 + c * alpha * dphi0)
    if (E_trial <= E0)
      break;
    alpha *= rho;
  }
  return alpha;
}

// Newton's method with custom tridiagonal solver
void Wire::newton_minimize(Eigen::VectorXd& theta,
                      int startId, int endId,
                      int max_iters,
                      double tol)
{
  std::vector<double> lower, diag, upper;

  for (int iter = 0; iter < max_iters; ++iter) {
    Eigen::VectorXd grad = compute_gradient(theta, startId, endId);
    double grad_norm = grad.norm();
    // std::cout << "Iter " << iter << ", ||grad|| = " << grad_norm << "\n";
    if (grad_norm < tol) break;

    compute_tridiagonal_hessian(theta, startId, endId, lower, diag, upper);

    // Solve H * delta = -grad
    Eigen::VectorXd direction;
    solve_tridiagonal(lower, diag, upper, -grad, direction);

    // Line search
    double step = line_search(theta, startId, endId, direction, grad);
    if ((step*direction).norm() < 1e-5) {
      break;  // Stop if line search step is too small
    }
    theta += step * direction;
  }
}

void Wire::updateSingleMatFrame(double theta, int idx) {
  nodes[idx].matframe.col(0) = edges[idx].bf.col(0);
    
  // twist y-axis of bishop frame by theta
  // about the x-axis of the bishop frame
  double cos_theta = std::cos(theta);
  double sin_theta = std::sin(theta);
  nodes[idx].matframe.col(1) = cos_theta * edges[idx].bf.col(1) + sin_theta * edges[idx].bf.col(2);

  // get z-axis of material frame by x.cross(y)
  nodes[idx].matframe.col(2) = nodes[idx].matframe.col(0).cross(nodes[idx].matframe.col(1));

  // make sure all axes are normalized
  nodes[idx].matframe.col(0).normalize();
  nodes[idx].matframe.col(1).normalize();
  nodes[idx].matframe.col(2).normalize();
}

void Wire::updateMatFrame() {
  // updates the materialframe of the edges
  // based on the twist angles 
  for (int i = 0; i < nv+1; i++) {
    // assign x-axis of bishop frame to x-axis of material frame
    nodes[i].matframe.col(0) = edges[i].bf.col(0);
    
    // twist y-axis of bishop frame by theta
    // about the x-axis of the bishop frame
    double cos_theta = std::cos(edges[i].theta);
    double sin_theta = std::sin(edges[i].theta);
    nodes[i].matframe.col(1) = cos_theta * edges[i].bf.col(1) + sin_theta * edges[i].bf.col(2);

    // get z-axis of material frame by x.cross(y)
    nodes[i].matframe.col(2) = nodes[i].matframe.col(0).cross(nodes[i].matframe.col(1));

    // make sure all axes are normalized
    nodes[i].matframe.col(0).normalize();
    nodes[i].matframe.col(1).normalize();
    nodes[i].matframe.col(2).normalize();
  }
}

void Wire::updateNewBWI(mjData* d, int idx) {
  // update theta_displace
  edges[idx].p_thetaloc = std::fmod(edges[idx].theta, (2. * M_PI));    
  if (edges[idx].p_thetaloc > M_PI) {
    edges[idx].p_thetaloc -= 2 * M_PI;
  }
  edges[idx].theta_displace = edges[idx].p_thetaloc;
  // update o2m
  updateO2M(d, idx);
}

void Wire::detectInteractions(const mjModel* m, mjData* d) {
  // interactions are detected before computation (before forward kinematics)
  // Detect interaction information in different ways:
  // 1. xfrc_applied
  // 2. qfrc_applied
  // 3. contacts
  // 4. equality/weldconstraints
  // remove or add as necessary.
  // no need to add first and last bodies to bodies_with_interactions
  // because they will be included in computation by default.

  // Store previous interactions before clearing
  std::vector<int> prev_bwi;
  prev_bwi = bodies_with_interactions;
  // Clear previous and new interaction
  bodies_with_interactions.clear();
  new_bwi.clear();

  // Create temporary sets to track which bodies have interactions
  std::unordered_set<int> temp_contacts;
  std::unordered_set<int> temp_forces;
  std::unordered_set<int> temp_constraints;
  
  double z_tol = 1e-7;
  // Create a set of all plugin body geom addresses for O(1) lookup
  std::unordered_set<int> plugin_body_addrs;
  for (int b = 1; b < n-1; b++) {
    plugin_body_addrs.insert(i0 + b);
    int body_id = i0 + b;
    // Check if any force or torque is applied to this body
    bool has_force = false;
    // 1. Check xfrc_applied (joint forces and torques) ======================
    for (int i = 0; i < 6; i++) {
      if (std::abs(d->xfrc_applied[6*body_id + i]) > z_tol) {  // Small threshold to avoid numerical noise
        has_force = true;
        break;
      }
    }
    // =======================================================================
    // 2. Check qfrc_applied (joint forces) ==================================
    if (!has_force && b < qvel_addrs.size()) {
      int qvel_addr = qvel_addrs[b];
      if (qvel_addr >= 0) {  // Check if joint exists
        for (int i = 0; i < joint_nv[b]; i++) {
          if (std::abs(d->qfrc_applied[qvel_addr + i]) > z_tol) {
            has_force = true;
            break;
          }
        }
      }
    }
    // =======================================================================
    if (has_force) {
      temp_forces.insert(body_id);
    }
  }
  
  // Check each contact once =================================================
  for (int i = 0; i < d->ncon; i++) {
    // Check if either geom in the contact belongs to our plugin bodies
    int body_id = m->geom_bodyid[d->contact[i].geom1];
    if (plugin_body_addrs.count(body_id)) {
      temp_contacts.insert(body_id);
    }
    body_id = m->geom_bodyid[d->contact[i].geom2];
    if (plugin_body_addrs.count(body_id)) {
      temp_contacts.insert(body_id);
    }
  }
  // =========================================================================

  // Check for active constraints ============================================
  for (int i = 0; i < m->neq; i++) {
    // Only check if the constraint is active and is a connect or weld constraint
    if (d->eq_active[i] && (m->eq_type[i] == mjEQ_CONNECT || m->eq_type[i] == mjEQ_WELD)) {
      // Get both bodies involved in the constraint
      int body_id = m->eq_obj1id[i];  // First body involved in the constraint
      if (plugin_body_addrs.count(body_id)) {
        temp_constraints.insert(body_id);
      }

      body_id = m->eq_obj2id[i];  // Second body involved in the constraint
      if (plugin_body_addrs.count(body_id)) {
        temp_constraints.insert(body_id);
      }
    }
  }
  // =========================================================================
  
  // Add bodies in contact, force, and equality/weldconstraints
  // in increasing order
  for (int b = 1; b < n-1; b++) {
    int body_id = i0 + b;
    if (temp_forces.count(body_id)) {
      bodies_with_interactions.push_back(b);
      break;
    }
    if (temp_contacts.count(body_id)) {
      bodies_with_interactions.push_back(b);
      break;
    }
    if (temp_constraints.count(body_id)) {
      bodies_with_interactions.push_back(b);
      break;
    }
  }

  // Find new interactions using set_difference
  std::set_difference(
      bodies_with_interactions.begin(), bodies_with_interactions.end(),
      prev_bwi.begin(), prev_bwi.end(),
      std::back_inserter(new_bwi)
  );
}

void Wire::printInfo(const mjModel* m, mjData* d) {
  // Print info:
  std::cout << "Compute" << std::endl;
  std::cout << "Number of Bodies (nbody): " << m->nbody << std::endl;
  for (int i=0; i < m->nbody; i++) {
    std::cout << "xfrc_applied[" << i << "]: ";
    // Print forces (first 3 elements)
    std::cout << "forces(" << d->xfrc_applied[6*i] << ", " 
              << d->xfrc_applied[6*i+1] << ", " 
              << d->xfrc_applied[6*i+2] << "), ";
    // Print torques (last 3 elements)
    std::cout << "torques(" << d->xfrc_applied[6*i+3] << ", " 
              << d->xfrc_applied[6*i+4] << ", " 
              << d->xfrc_applied[6*i+5] << ")" << std::endl;
  }
}

// Print timing info at plugin destruction
void Wire::PrintComputeTiming() {
  if (!timing_enabled) return;
  std::cout << "[Wire] Compute called " << compute_call_count << " times. "
            << "Total time: " << total_compute_time_ms << " ms. "
            << "Average time: " << (compute_call_count ? (total_compute_time_ms / compute_call_count) : 0.0) << " ms." << std::endl;
}

}  // namespace mujoco::plugin::elasticity
