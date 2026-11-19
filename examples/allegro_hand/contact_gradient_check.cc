// Moving-contact gradient check on the Allegro hand's planning model, with IDTO's own (smooth, compliant) contact
// model and the parameters of allegro_hand.yaml.
//
//  1. Finds a non-degenerate sphere-sphere contact between a finger sphere and the ball and records its geometry:
//     pair ids and names, body-local and world witness points, normal, signed distance.
//  2. Perturbs a finger joint and recomputes the geometry: the witness points must slide along both spheres
//     (constant distance from each centre), as the closed-form sphere-sphere closest points predict.
//  3. Compares IDTO's configured inverse-dynamics derivative d tau[t] / d q[t+1] (the example's gradients_method)
//     with independent central differences of tau over several step sizes: per finger joint, for the ball's position,
//     and for the ball's orientation along valid unit-quaternion curves q(e) = q (x) exp(e w / 2).
//  4. Splits the generalized contact force J_c(q)^T f_c(q, v): the contact point C(q) (midpoint of the witness points)
//     moving with q, against the same force applied at body-fixed material points frozen at the nominal C.
//  5. Distinguishes the geometric motion of the contact point dC/dt from the contacting material points' velocities,
//     which IDTO uses for dissipation and friction.
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "examples/yaml_config.h"
#include "optimizer/problem_definition.h"
#include "optimizer/solver_parameters.h"
#include "optimizer/trajectory_optimizer.h"
#include "utils/find_resource.h"
#include <drake/common/fmt_eigen.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/geometry/query_object.h>
#include <drake/geometry/scene_graph_inspector.h>
#include <drake/geometry/shape_specification.h>
#include <drake/math/rigid_transform.h>
#include <drake/math/roll_pitch_yaw.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/plant/multibody_plant_config_functions.h>
#include <drake/multibody/tree/multibody_forces.h>
#include <drake/systems/framework/diagram_builder.h>
#include <fmt/format.h>
#include <gflags/gflags.h>

DEFINE_double(ball_spin, 0.0,
              "Ball angular velocity about world-aligned x between q[1] and q[2] [rad/s]; nonzero exposes the "
              "q-dependence of N+(q) that forward differences hold fixed");
DEFINE_string(gradients_method, "",
              "Override the YAML gradients_method (forward_differences, central_differences, central_differences4, "
              "autodiff)");

namespace idto {
namespace examples {
namespace allegro_hand {

using drake::fmt_eigen;
using drake::geometry::GeometryId;
using drake::geometry::QueryObject;
using drake::geometry::SceneGraphInspector;
using drake::geometry::SignedDistancePair;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::multibody::AddMultibodyPlant;
using drake::multibody::CoulombFriction;
using drake::multibody::JacobianWrtVariable;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyForces;
using drake::multibody::MultibodyPlant;
using drake::multibody::MultibodyPlantConfig;
using drake::multibody::Parser;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::SpatialVelocity;
using drake::multibody::UnitInertia;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;
using optimizer::GradientsMethod;
using optimizer::ProblemDefinition;
using optimizer::SolverParameters;
using optimizer::TrajectoryOptimizer;
using optimizer::TrajectoryOptimizerState;
using utils::FindIdtoResource;

constexpr double kBallRadius = 0.06;

// The planning model of allegro_hand.cc (CreatePlantModel), without the visual-only geometry.
void CreatePlanningModel(MultibodyPlant<double>* plant) {
  Parser(plant).AddModels(FindIdtoResource("idto/models/allegro_hand.sdf"));
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("hand_root"),
                    RigidTransformd(RollPitchYawd(0, -M_PI_2, 0), Vector3d::Zero()));
  const ModelInstanceIndex ball_idx = plant->AddModelInstance("ball");
  const SpatialInertia<double> I(0.05, Vector3d::Zero(), UnitInertia<double>::SolidSphere(kBallRadius));
  const RigidBody<double>& ball = plant->AddRigidBody("ball", ball_idx, I);
  plant->RegisterCollisionGeometry(ball, RigidTransformd::Identity(), Sphere(kBallRadius), "ball_collision",
                                   CoulombFriction<double>());
}

// One contact pair as IDTO's CalcContactForceContribution sees it, with the force it applies.
struct ContactSample {
  GeometryId id_A, id_B;
  std::string name_A, name_B;
  const RigidBody<double>* body_A{};
  const RigidBody<double>* body_B{};
  double phi{};
  Vector3d nhat_W;              // normal, outwards from A
  Vector3d p_WCa, p_WCb, p_WC;  // witness points and their midpoint C, world
  Vector3d p_GaCa, p_GbCb;      // witness points in their geometry frames
  Vector3d p_AoC_A, p_BoC_B;    // C in the body frames
  Vector3d v_AcBc_W;            // velocity of B's material point at C relative to A's
  Vector3d f_BC_W;              // force on B at C
};

double ForceThreshold(const SolverParameters& p) {
  const double eps = std::sqrt(std::numeric_limits<double>::epsilon());
  return -p.smoothing_factor * std::log(std::exp(eps / (p.smoothing_factor * p.contact_stiffness)) - 1.0);
}

// Replicates IDTO's force law (trajectory_optimizer.cc, CalcContactForceContribution) per pair.
std::vector<ContactSample> EvalContacts(const MultibodyPlant<double>& plant, const Context<double>& context,
                                        const SolverParameters& p, double threshold) {
  const auto& query = plant.get_geometry_query_input_port().Eval<QueryObject<double>>(context);
  const SceneGraphInspector<double>& inspector = query.inspector();
  std::vector<ContactSample> out;
  for (const SignedDistancePair<double>& pair : query.ComputeSignedDistancePairwiseClosestPoints(threshold)) {
    ContactSample c;
    c.id_A = pair.id_A;
    c.id_B = pair.id_B;
    c.name_A = inspector.GetName(pair.id_A);
    c.name_B = inspector.GetName(pair.id_B);
    c.body_A = plant.GetBodyFromFrameId(inspector.GetFrameId(pair.id_A));
    c.body_B = plant.GetBodyFromFrameId(inspector.GetFrameId(pair.id_B));
    c.phi = pair.distance;
    c.nhat_W = -pair.nhat_BA_W;
    const RigidTransformd& X_WA = plant.EvalBodyPoseInWorld(context, *c.body_A);
    const RigidTransformd& X_WB = plant.EvalBodyPoseInWorld(context, *c.body_B);
    c.p_GaCa = pair.p_ACa;
    c.p_GbCb = pair.p_BCb;
    c.p_WCa = X_WA * inspector.GetPoseInFrame(pair.id_A) * pair.p_ACa;
    c.p_WCb = X_WB * inspector.GetPoseInFrame(pair.id_B) * pair.p_BCb;
    c.p_WC = 0.5 * (c.p_WCa + c.p_WCb);
    c.p_AoC_A = X_WA.inverse() * c.p_WC;
    c.p_BoC_B = X_WB.inverse() * c.p_WC;
    const SpatialVelocity<double>& V_WA = plant.EvalBodySpatialVelocityInWorld(context, *c.body_A);
    const SpatialVelocity<double>& V_WB = plant.EvalBodySpatialVelocityInWorld(context, *c.body_B);
    c.v_AcBc_W = V_WB.Shift(c.p_WC - X_WB.translation()).translational() -
                 V_WA.Shift(c.p_WC - X_WA.translation()).translational();
    const double vn = c.nhat_W.dot(c.v_AcBc_W);
    const Vector3d vt = c.v_AcBc_W - vn * c.nhat_W;
    const double s = vn / p.dissipation_velocity;
    const double dissipation = s < 0 ? 1 - s : (s < 2 ? (s - 2) * (s - 2) / 4 : 0.0);
    const double exponent = -c.phi / p.smoothing_factor;
    const double compliant =
        exponent >= 37 ? -p.contact_stiffness * c.phi
                       : p.smoothing_factor * p.contact_stiffness * std::log(1 + std::exp(exponent));
    const double fn = compliant * dissipation;
    const Vector3d t_reg = -vt / std::sqrt(p.stiction_velocity * p.stiction_velocity + vt.squaredNorm());
    c.f_BC_W = c.nhat_W * fn + t_reg * p.friction_coefficient * fn;
    out.push_back(c);
  }
  return out;
}

// sum over pairs of J_B(C)^T f - J_A(C)^T f, with C either the current contact point or (frozen) the body-fixed
// points given per pair.
VectorXd GeneralizedContactForce(const MultibodyPlant<double>& plant, const Context<double>& context,
                                 const std::vector<ContactSample>& contacts,
                                 const std::vector<std::pair<Vector3d, Vector3d>>* frozen = nullptr) {
  VectorXd tau = VectorXd::Zero(plant.num_velocities());
  MatrixXd J(3, plant.num_velocities());
  for (size_t i = 0; i < contacts.size(); ++i) {
    const ContactSample& c = contacts[i];
    const Vector3d p_A = frozen ? (*frozen)[i].first : c.p_AoC_A;
    const Vector3d p_B = frozen ? (*frozen)[i].second : c.p_BoC_B;
    plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable::kV, c.body_B->body_frame(), p_B,
                                            plant.world_frame(), plant.world_frame(), &J);
    tau += J.transpose() * c.f_BC_W;
    plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable::kV, c.body_A->body_frame(), p_A,
                                            plant.world_frame(), plant.world_frame(), &J);
    tau -= J.transpose() * c.f_BC_W;
  }
  return tau;
}

GradientsMethod ParseGradientsMethod(const std::string& name) {
  if (name == "forward_differences") return GradientsMethod::kForwardDifferences;
  if (name == "central_differences") return GradientsMethod::kCentralDifferences;
  if (name == "central_differences4") return GradientsMethod::kCentralDifferences4;
  if (name == "autodiff") return GradientsMethod::kAutoDiff;
  throw std::runtime_error("unknown gradients_method " + name);
}

// q with the ball's quaternion rotated by exp(e w / 2) (a valid unit-quaternion curve).
VectorXd RotateBall(const VectorXd& q, int quat_start, const Vector3d& w, double e) {
  VectorXd out = q;
  const Quaterniond quat(q(quat_start), q(quat_start + 1), q(quat_start + 2), q(quat_start + 3));
  const double angle = e * w.norm();
  const Quaterniond dq(Eigen::AngleAxisd(angle, w.normalized()));
  const Quaterniond rotated = (quat * dq).normalized();
  out.segment<4>(quat_start) << rotated.w(), rotated.x(), rotated.y(), rotated.z();
  return out;
}

int Run() {
  TrajOptExampleParams options = drake::yaml::LoadYamlFile<TrajOptExampleParams>(
      FindIdtoResource("idto/examples/allegro_hand/allegro_hand.yaml"), {}, TrajOptExampleParams());
  const std::string method = FLAGS_gradients_method.empty() ? options.gradients_method : FLAGS_gradients_method;

  DiagramBuilder<double> builder;
  MultibodyPlantConfig config;
  config.time_step = options.time_step;
  auto [plant, scene_graph] = AddMultibodyPlant(config, &builder);
  CreatePlanningModel(&plant);
  plant.Finalize();
  auto diagram = builder.Build();
  const int nq = plant.num_positions();
  const int nv = plant.num_velocities();
  const RigidBody<double>& ball = plant.GetBodyByName("ball");
  const int ball_q = ball.floating_positions_start();  // [qw qx qy qz x y z]
  const double dt = options.time_step;

  SolverParameters params;
  params.contact_stiffness = options.contact_stiffness;
  params.dissipation_velocity = options.dissipation_velocity;
  params.friction_coefficient = options.friction_coefficient;
  params.stiction_velocity = options.stiction_velocity;
  params.smoothing_factor = options.smoothing_factor;
  params.gradients_method = ParseGradientsMethod(method);
  params.equality_constraints = options.equality_constraints;
  params.num_threads = 1;
  params.q_nom_relative_to_q_init = VectorX<bool>::Constant(nq, false);
  const double threshold = ForceThreshold(params);

  std::cout << std::setprecision(6);
  std::cout << fmt::format("Allegro planning model: nq {} nv {}, time_step {}, gradients_method {}, ball spin {} rad/s\n",
                           nq, nv, dt, method, FLAGS_ball_spin);
  std::cout << fmt::format("contact: k {} N/m, sigma {} m, dissipation velocity {} m/s, mu {}, stiction {} m/s; "
                           "force threshold distance {:.4g} m\n",
                           params.contact_stiffness, params.smoothing_factor, params.dissipation_velocity,
                           params.friction_coefficient, params.stiction_velocity, threshold);

  // Scratch context for geometry-only queries.
  auto diagram_context = diagram->CreateDefaultContext();
  Context<double>& ctx = diagram->GetMutableSubsystemContext(plant, diagram_context.get());
  VectorXd q_init = options.q_init;
  q_init.segment<4>(ball_q).normalize();
  plant.SetPositions(&ctx, q_init);
  plant.SetVelocities(&ctx, VectorXd::Zero(nv));

  // --- 1. the closest finger-ball pair at q_init, then press that finger 1 mm into the ball ---
  // Geometry names are scoped by model instance (ball::ball_collision), so match bodies and geometry ids.
  auto is_ball = [](const ContactSample& c) { return c.body_A->name() == "ball" || c.body_B->name() == "ball"; };
  auto finger_ball = [&](const VectorXd& q) {
    plant.SetPositions(&ctx, q);
    std::vector<ContactSample> best;
    for (const ContactSample& c : EvalContacts(plant, ctx, params, 0.05)) {
      // Ties (mirror-image fingers) broken by name, so every run picks the same pair.
      if (is_ball(c) && (best.empty() || c.phi < best[0].phi - 1e-12 ||
                         (std::abs(c.phi - best[0].phi) <= 1e-12 && c.name_A + c.name_B < best[0].name_A + best[0].name_B)))
        best = {c};
    }
    if (best.empty()) throw std::runtime_error("no finger-ball pair within 5 cm");
    return best[0];
  };
  ContactSample nearest = finger_ball(q_init);
  const bool ball_is_A = nearest.body_A->name() == "ball";
  const GeometryId finger_id = ball_is_A ? nearest.id_B : nearest.id_A;
  const std::string finger_geometry = ball_is_A ? nearest.name_B : nearest.name_A;
  auto involves_finger = [&](const ContactSample& c) { return c.id_A == finger_id || c.id_B == finger_id; };
  auto phi_of = [&](const VectorXd& q) {
    plant.SetPositions(&ctx, q);
    for (const ContactSample& c : EvalContacts(plant, ctx, params, 0.05)) {
      if (involves_finger(c) && is_ball(c)) return c.phi;
    }
    return 0.05;
  };
  std::cout << fmt::format("\nnearest finger-ball pair at q_init: {} - {}, phi {:.5f} m\n", nearest.name_A,
                           nearest.name_B, nearest.phi);

  // Hand joint coordinates are the q entries outside the ball's 7 floating coordinates.
  std::vector<int> hand_q;
  for (int i = 0; i < nq; ++i) {
    if (i < ball_q || i >= ball_q + 7) hand_q.push_back(i);
  }
  int press = -1;
  double best_slope = 0;
  for (int i : hand_q) {
    VectorXd qp = q_init, qm = q_init;
    qp(i) += 1e-4;
    qm(i) -= 1e-4;
    const double slope = (phi_of(qp) - phi_of(qm)) / 2e-4;
    if (std::abs(slope) > std::abs(best_slope)) {
      best_slope = slope;
      press = i;
    }
  }
  VectorXd q_contact = q_init;
  for (int k = 0; k < 20; ++k) {  // Newton on phi(q_press) = -1 mm
    const double phi = phi_of(q_contact);
    if (std::abs(phi + 1e-3) < 1e-7) break;
    VectorXd qp = q_contact, qm = q_contact;
    qp(press) += 1e-5;
    qm(press) -= 1e-5;
    q_contact(press) -= (phi + 1e-3) / ((phi_of(qp) - phi_of(qm)) / 2e-5);
  }
  std::cout << fmt::format("pressing joint q[{}] moved {:+.4f} rad: phi {:.6f} m\n", press,
                           q_contact(press) - q_init(press), phi_of(q_contact));

  // The joint that slides the contact the most along the ball (tangential witness motion on the ball).
  auto ball_witness = [&](const VectorXd& q) {
    plant.SetPositions(&ctx, q);
    for (const ContactSample& c : EvalContacts(plant, ctx, params, 0.05)) {
      if (c.id_A == finger_id && is_ball(c)) return Vector3d(c.p_GbCb);
      if (c.id_B == finger_id && is_ball(c)) return Vector3d(c.p_GaCa);
    }
    return Vector3d(Vector3d::Constant(NAN));
  };
  int slide = -1;
  double best_tangential = 0;
  {
    const Vector3d p0 = ball_witness(q_contact);
    const Vector3d n0 = p0.normalized();
    for (int i : hand_q) {
      VectorXd qp = q_contact;
      qp(i) += 1e-3;
      const Vector3d dp = (ball_witness(qp) - p0) / 1e-3;
      const double tangential = (dp - dp.dot(n0) * n0).norm();
      if (tangential > best_tangential) {
        best_tangential = tangential;
        slide = i;
      }
    }
  }
  std::cout << fmt::format("sliding joint q[{}]: tangential witness speed on the ball {:.4f} m/rad\n", slide,
                           best_tangential);

  // Trajectory for tau[1] = ID(q[2], v[2], a[1]): the finger slides along the ball between q[1] and q[2].
  const int num_steps = 2;
  std::vector<VectorXd> q(num_steps + 1, q_contact);
  q[0](slide) -= 0.02;
  q[1](slide) -= 0.02;
  if (FLAGS_ball_spin != 0.0) {  // the ball turns by ball_spin * dt between q[1] and q[2] (and q[0] -> q[1])
    q[1] = RotateBall(q[1], ball_q, Vector3d::UnitX(), -FLAGS_ball_spin * dt);
    q[0] = RotateBall(q[1], ball_q, Vector3d::UnitX(), -FLAGS_ball_spin * dt);
  }

  ProblemDefinition prob;
  prob.num_steps = num_steps;
  prob.q_init = q[0];
  prob.v_init = VectorXd::Zero(nv);
  prob.Qq = MatrixXd::Identity(nq, nq);
  prob.Qv = MatrixXd::Identity(nv, nv);
  prob.Qf_q = MatrixXd::Identity(nq, nq);
  prob.Qf_v = MatrixXd::Identity(nv, nv);
  prob.R = MatrixXd::Identity(nv, nv);
  prob.q_nom.assign(num_steps + 1, q[0]);
  prob.v_nom.assign(num_steps + 1, VectorXd::Zero(nv));
  TrajectoryOptimizer<double> optimizer(diagram.get(), &plant, prob, params);
  TrajectoryOptimizerState<double> state = optimizer.CreateState();
  auto tau1 = [&](const VectorXd& q2) {
    std::vector<VectorXd> qq = q;
    qq[2] = q2;
    state.set_q(qq);
    return VectorXd(optimizer.EvalTau(state)[1]);
  };
  state.set_q(q);
  const VectorXd tau_nominal = optimizer.EvalTau(state)[1];
  const MatrixXd dtau_dq2 = optimizer.EvalInverseDynamicsPartials(state).dtau_dqp[1];
  const VectorXd v2 = optimizer.EvalV(state)[2];
  const VectorXd a1 = optimizer.EvalA(state)[1];

  // --- contact table at q[2] (with the trajectory's velocity v[2]) ---
  const Context<double>& ctx2 = optimizer.EvalPlantContext(state, 2);
  const std::vector<ContactSample> contacts = EvalContacts(plant, ctx2, params, threshold);
  std::cout << fmt::format("\n1. Contact pairs within the force threshold at q[2] ({} pairs):\n", contacts.size());
  for (const ContactSample& c : contacts) {
    std::cout << fmt::format(
        "  [{} | {}] bodies {} / {}, phi {:+.6f} m, n_W {}, p_WC {}\n"
        "      p_GaCa {}  p_GbCb {}  (witness points in their geometry frames)\n"
        "      v_AcBc_W {}  f_BC_W {} N\n",
        c.name_A, c.name_B, c.body_A->name(), c.body_B->name(), c.phi, fmt_eigen(c.nhat_W.transpose()),
        fmt_eigen(c.p_WC.transpose()), fmt_eigen(c.p_GaCa.transpose()), fmt_eigen(c.p_GbCb.transpose()),
        fmt_eigen(c.v_AcBc_W.transpose()), fmt_eigen(c.f_BC_W.transpose()));
  }

  // --- 2. witness points slide along both spheres as the finger joint moves ---
  std::cout << fmt::format("\n2. Witness points of {} | ball_collision under q[{}] perturbations:\n",
                           finger_geometry, slide);
  std::cout << "     dq [rad]   phi [m]    |p_ball|-r [m]  ball-point arc [mm]  |p_finger|-r_f [m]  "
               "closed-form error [m]\n";
  {
    plant.SetPositions(&ctx, q[2]);
    const auto& query = plant.get_geometry_query_input_port().Eval<QueryObject<double>>(ctx);
    const SceneGraphInspector<double>& inspector = query.inspector();
    const double r_f = dynamic_cast<const Sphere&>(inspector.GetShape(finger_id)).radius();
    const Vector3d p0 = ball_witness(q[2]);
    for (double h : {-2e-2, -1e-2, -1e-3, 0.0, 1e-3, 1e-2, 2e-2}) {
      VectorXd qh = q[2];
      qh(slide) += h;
      plant.SetPositions(&ctx, qh);
      const auto& qo = plant.get_geometry_query_input_port().Eval<QueryObject<double>>(ctx);
      const auto pair = qo.ComputeSignedDistancePairClosestPoints(finger_id, inspector.GetGeometries(
          plant.GetBodyFrameIdOrThrow(ball.index()))[0]);
      const Vector3d p_ball = pair.id_A == finger_id ? pair.p_BCb : pair.p_ACa;   // ball geometry frame
      const Vector3d p_fing = pair.id_A == finger_id ? pair.p_ACa : pair.p_BCb;   // finger sphere frame
      // Closed form: the ball point lies on the ball surface towards the finger sphere centre.
      const RigidTransformd X_WBall = plant.EvalBodyPoseInWorld(ctx, ball);
      const RigidTransformd X_WF = qo.GetPoseInWorld(finger_id);
      const Vector3d c_f_ball = X_WBall.inverse() * X_WF.translation();
      const Vector3d expected = kBallRadius * c_f_ball.normalized();
      const double arc = 1e3 * kBallRadius * std::acos(std::clamp(p_ball.normalized().dot(p0.normalized()), -1.0, 1.0));
      std::cout << fmt::format("    {:+.4f}  {:+.6f}  {:+.2e}       {:8.3f}            {:+.2e}          {:.2e}\n", h,
                               pair.distance, p_ball.norm() - kBallRadius, arc, p_fing.norm() - r_f,
                               (p_ball - expected).norm());
    }
  }

  // --- 3. configured derivative vs. central differences ---
  std::cout << fmt::format("\n3. d tau[1] / d q[2]: {} (as configured) vs. central differences of tau\n", method);
  std::cout << "   direction                 |D_cfg d|      rel. error vs central difference at h =\n";
  std::cout << "                                           1e-2      1e-3      1e-4      1e-5      1e-6      1e-7\n";
  auto report = [&](const std::string& label, const VectorXd& D_dir, auto curve) {
    std::cout << fmt::format("   {:24s}  {:10.4g}", label, D_dir.norm());
    for (double h : {1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7}) {
      const VectorXd cd = (tau1(curve(h)) - tau1(curve(-h))) / (2 * h);
      std::cout << fmt::format("  {:8.2e}", (D_dir - cd).norm() / std::max(cd.norm(), 1e-12));
    }
    std::cout << "\n";
  };
  for (int i : {slide, press}) {
    VectorXd e = VectorXd::Zero(nq);
    e(i) = 1;
    report(fmt::format("hand q[{}]{}", i, i == slide ? " (sliding)" : " (pressing)"), dtau_dq2 * e,
           [&](double h) { VectorXd x = q[2]; x(i) += h; return x; });
  }
  {
    // All hand joints at once: Frobenius norm of the column block.
    std::cout << "   all hand joints (matrix)              ";
    for (double h : {1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7}) {
      double num = 0, den = 0;
      for (int i : hand_q) {
        VectorXd xp = q[2], xm = q[2];
        xp(i) += h;
        xm(i) -= h;
        const VectorXd cd = (tau1(xp) - tau1(xm)) / (2 * h);
        num += (dtau_dq2.col(i) - cd).squaredNorm();
        den += cd.squaredNorm();
      }
      std::cout << fmt::format("  {:8.2e}", std::sqrt(num / den));
    }
    std::cout << "\n";
  }
  for (int k = 0; k < 3; ++k) {
    const int i = ball_q + 4 + k;
    VectorXd e = VectorXd::Zero(nq);
    e(i) = 1;
    report(fmt::format("ball position {}", "xyz"[k]), dtau_dq2 * e,
           [&](double h) { VectorXd x = q[2]; x(i) += h; return x; });
  }
  for (int k = 0; k < 3; ++k) {
    const Vector3d w = Vector3d::Unit(k);
    // d q / d e at e = 0 along q (x) exp(e w / 2): 0.5 * quat (x) (0, w)
    const Quaterniond quat(q[2](ball_q), q[2](ball_q + 1), q[2](ball_q + 2), q[2](ball_q + 3));
    const Quaterniond qdot = quat * Quaterniond(0, 0.5 * w.x(), 0.5 * w.y(), 0.5 * w.z());
    VectorXd e = VectorXd::Zero(nq);
    e.segment<4>(ball_q) << qdot.w(), qdot.x(), qdot.y(), qdot.z();
    report(fmt::format("ball rotation {} (unit q)", "xyz"[k]), dtau_dq2 * e,
           [&](double h) { return RotateBall(q[2], ball_q, w, h); });
  }

  // --- 4. J_c(q)^T f_c(q, v): moving contact point vs. frozen material points ---
  std::cout << "\n4. Generalized contact force J_c(q)^T f_c(q, v) at tau[1]\n";
  state.set_q(q);  // back to the nominal trajectory after the perturbations above
  {
    // IDTO's own: tau = ID_no_contact - J^T f, so J^T f = ID(q, v, a; gravity only) - tau.
    const Context<double>& c2 = optimizer.EvalPlantContext(state, 2);
    MultibodyForces<double> gravity(plant);
    plant.CalcForceElementsContribution(c2, &gravity);
    const VectorXd jtf_idto = plant.CalcInverseDynamics(c2, a1, gravity) - tau_nominal;
    const VectorXd jtf_replica = GeneralizedContactForce(plant, c2, contacts);
    std::cout << fmt::format("   replica of IDTO's force law vs IDTO's inverse dynamics: |J^T f| {:.6g}, "
                             "difference {:.2e}\n",
                             jtf_idto.norm(), (jtf_idto - jtf_replica).norm());
  }
  {
    std::vector<std::pair<Vector3d, Vector3d>> frozen;
    for (const ContactSample& c : contacts) frozen.push_back({c.p_AoC_A, c.p_BoC_B});
    auto jtf_at = [&](const VectorXd& qx, bool freeze) {
      plant.SetPositions(&ctx, qx);
      plant.SetVelocities(&ctx, v2);
      std::vector<ContactSample> cs = EvalContacts(plant, ctx, params, threshold);
      // Keep the nominal pairs only, in the nominal order.
      std::vector<ContactSample> matched;
      for (const ContactSample& c0 : contacts) {
        for (const ContactSample& c : cs) {
          if (c.id_A == c0.id_A && c.id_B == c0.id_B) matched.push_back(c);
        }
      }
      DRAKE_DEMAND(matched.size() == contacts.size());
      return GeneralizedContactForce(plant, ctx, matched, freeze ? &frozen : nullptr);
    };
    const double h = 1e-5;
    for (int i : {slide, press}) {
      VectorXd xp = q[2], xm = q[2];
      xp(i) += h;
      xm(i) -= h;
      const VectorXd moving = (jtf_at(xp, false) - jtf_at(xm, false)) / (2 * h);
      const VectorXd fixed = (jtf_at(xp, true) - jtf_at(xm, true)) / (2 * h);
      std::cout << fmt::format(
          "   d(J^T f)/dq[{}]: contact point moving with q {:.5g}, frozen at the nominal material points {:.5g}, "
          "difference (lever-arm term) {:.5g} ({:.1f}%)\n",
          i, moving.norm(), fixed.norm(), (moving - fixed).norm(), 100 * (moving - fixed).norm() / moving.norm());
    }
  }

  // --- 5. geometric contact-point motion vs. material-point velocities ---
  std::cout << "\n5. Contact point motion along the trajectory (q[1] -> q[2], the finger sliding):\n";
  for (const ContactSample& c : contacts) {
    if (!involves_finger(c)) continue;
    auto contact_point = [&](const VectorXd& qx) {
      plant.SetPositions(&ctx, qx);
      for (const ContactSample& s : EvalContacts(plant, ctx, params, 0.05)) {
        if (s.id_A == c.id_A && s.id_B == c.id_B) return s.p_WC;
      }
      return Vector3d(Vector3d::Constant(NAN));
    };
    // q_dot = v for the hand joints; the ball does not move in this trajectory.
    VectorXd qdot = VectorXd::Zero(nq);
    for (int i : hand_q) qdot(i) = (q[2](i) - q[1](i)) / dt;
    const double h = 1e-6;
    const Vector3d dC_dt = (contact_point(q[2] + h * qdot) - contact_point(q[2] - h * qdot)) / (2 * h);
    plant.SetPositions(&ctx, q[2]);
    plant.SetVelocities(&ctx, v2);
    const SpatialVelocity<double>& V_WA = plant.EvalBodySpatialVelocityInWorld(ctx, *c.body_A);
    const SpatialVelocity<double>& V_WB = plant.EvalBodySpatialVelocityInWorld(ctx, *c.body_B);
    const Vector3d v_Ac =
        V_WA.Shift(c.p_WC - plant.EvalBodyPoseInWorld(ctx, *c.body_A).translation()).translational();
    const Vector3d v_Bc =
        V_WB.Shift(c.p_WC - plant.EvalBodyPoseInWorld(ctx, *c.body_B).translation()).translational();
    std::cout << fmt::format(
        "   {} | {}: dC/dt {} m/s (|.| {:.4f}); material points v_Ac {} (|.| {:.4f}), v_Bc {} (|.| {:.4f});\n"
        "   slip v_AcBc {} m/s, normal part {:+.2e}: IDTO's friction and dissipation use the slip, not dC/dt\n",
        c.name_A, c.name_B, fmt_eigen(dC_dt.transpose()), dC_dt.norm(), fmt_eigen(v_Ac.transpose()), v_Ac.norm(),
        fmt_eigen(v_Bc.transpose()), v_Bc.norm(), fmt_eigen((v_Bc - v_Ac).transpose()),
        c.nhat_W.dot(v_Bc - v_Ac));
  }
  return 0;
}

}  // namespace allegro_hand
}  // namespace examples
}  // namespace idto

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return idto::examples::allegro_hand::Run();
}
