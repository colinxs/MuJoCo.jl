
# TODO overload print/show/display function to not display model and data

struct jlModel
   m::Ref{mjModel}

   qpos0::Vector{mjtNum}
   qpos_spring::Vector{mjtNum}
   body_parentid::Vector{Cint}
   body_rootid::Vector{Cint}
   body_weldid::Vector{Cint}
   body_mocapid::Vector{Cint}
   body_jntnum::Vector{Cint}
   body_jntadr::Vector{Cint}
   body_dofnum::Vector{Cint}
   body_dofadr::Vector{Cint}
   body_geomnum::Vector{Cint}
   body_geomadr::Vector{Cint}
   body_simple::Vector{mjtByte}
   body_sameframe::Vector{mjtByte}
   body_pos::Matrix{mjtNum}
   body_quat::Matrix{mjtNum}
   body_ipos::Matrix{mjtNum}
   body_iquat::Matrix{mjtNum}
   body_mass::Vector{mjtNum}
   body_subtreemass::Vector{mjtNum}
   body_inertia::Matrix{mjtNum}
   body_invweight0::Matrix{mjtNum}
   body_user::Matrix{mjtNum}
   jnt_type::Vector{Cint}
   jnt_qposadr::Vector{Cint}
   jnt_dofadr::Vector{Cint}
   jnt_bodyid::Vector{Cint}
   jnt_group::Vector{Cint}
   jnt_limited::Vector{mjtByte}
   jnt_solref::Matrix{mjtNum}
   jnt_solimp::Matrix{mjtNum}
   jnt_pos::Matrix{mjtNum}
   jnt_axis::Matrix{mjtNum}
   jnt_stiffness::Vector{mjtNum}
   jnt_range::Matrix{mjtNum}
   jnt_margin::Vector{mjtNum}
   jnt_user::Matrix{mjtNum}

   dof_bodyid::Vector{Cint}
   dof_jntid::Vector{Cint}
   dof_parentid::Vector{Cint}
   dof_Madr::Vector{Cint}
   dof_simplenum::Vector{Cint}
   dof_solref::Matrix{mjtNum}
   dof_solimp::Matrix{mjtNum}
   dof_frictionloss::Vector{mjtNum}
   dof_armature::Vector{mjtNum}
   dof_damping::Vector{mjtNum}
   dof_invweight0::Vector{mjtNum}
   dof_M0::Vector{mjtNum}

   geom_type::Vector{Cint}
   geom_contype::Vector{Cint}
   geom_conaffinity::Vector{Cint}
   geom_condim::Vector{Cint}
   geom_bodyid::Vector{Cint}
   geom_dataid::Vector{Cint}
   geom_matid::Vector{Cint}
   geom_group::Vector{Cint}
   geom_priority::Vector{Cint}
   geom_sameframe::Vector{mjtByte}
   geom_solmix::Vector{mjtNum}
   geom_solref::Matrix{mjtNum}
   geom_solimp::Matrix{mjtNum}
   geom_size::Matrix{mjtNum}
   geom_rbound::Vector{mjtNum}
   geom_pos::Matrix{mjtNum}
   geom_quat::Matrix{mjtNum}
   geom_friction::Matrix{mjtNum}
   geom_margin::Vector{mjtNum}
   geom_gap::Vector{mjtNum}
   geom_user::Matrix{mjtNum}
   geom_rgba::Matrix{Cfloat}

   site_type::Vector{Cint}
   site_bodyid::Vector{Cint}
   site_matid::Vector{Cint}
   site_group::Vector{Cint}
   site_sameframe::Vector{mjtByte}
   site_size::Matrix{mjtNum}
   site_pos::Matrix{mjtNum}
   site_quat::Matrix{mjtNum}
   site_user::Matrix{mjtNum}
   site_rgba::Matrix{Cfloat}

   cam_mode::Vector{Cint}
   cam_bodyid::Vector{Cint}
   cam_targetbodyid::Vector{Cint}
   cam_pos::Matrix{mjtNum}
   cam_quat::Matrix{mjtNum}
   cam_poscom0::Matrix{mjtNum}
   cam_pos0::Matrix{mjtNum}
   cam_mat0::Matrix{mjtNum}
   cam_fovy::Vector{mjtNum}
   cam_ipd::Vector{mjtNum}
   cam_user::Matrix{mjtNum}

   light_mode::Vector{Cint}
   light_bodyid::Vector{Cint}
   light_targetbodyid::Vector{Cint}
   light_directional::Vector{mjtByte}
   light_castshadow::Vector{mjtByte}
   light_active::Vector{mjtByte}
   light_pos::Matrix{mjtNum}
   light_dir::Matrix{mjtNum}
   light_poscom0::Matrix{mjtNum}
   light_pos0::Matrix{mjtNum}
   light_dir0::Matrix{mjtNum}
   light_attenuation::Matrix{Cfloat}
   light_cutoff::Vector{Cfloat}
   light_exponent::Vector{Cfloat}
   light_ambient::Matrix{Cfloat}
   light_diffuse::Matrix{Cfloat}
   light_specular::Matrix{Cfloat}

   mesh_vertadr::Vector{Cint}
   mesh_vertnum::Vector{Cint}
   mesh_texcoordadr::Vector{Cint}
   mesh_faceadr::Vector{Cint}
   mesh_facenum::Vector{Cint}
   mesh_graphadr::Vector{Cint}
   mesh_vert::Matrix{Cfloat}
   mesh_normal::Matrix{Cfloat}
   mesh_texcoord::Matrix{Cfloat}
   mesh_face::Matrix{Cint}
   mesh_graph::Vector{Cint}

   skin_matid::Vector{Cint}
   skin_rgba::Matrix{Float32}
   skin_inflate::Vector{Float32}
   skin_vertadr::Vector{Cint}
   skin_vertnum::Vector{Cint}
   skin_texcoordadr::Vector{Cint}
   skin_faceadr::Vector{Cint}
   skin_facenum::Vector{Cint}
   skin_boneadr::Vector{Cint}
   skin_bonenum::Vector{Cint}
   skin_vert::Matrix{Float32}
   skin_texcoord::Matrix{Float32}
   skin_face::Matrix{Cint}
   skin_bonevertadr::Vector{Cint}
   skin_bonevertnum::Vector{Cint}
   skin_bonebindpos::Matrix{Float32}
   skin_bonebindquat::Matrix{Float32}
   skin_bonebodyid::Vector{Cint}
   skin_bonevertid::Vector{Cint}
   skin_bonevertweight::Vector{Float32}

   hfield_size::Matrix{mjtNum}
   hfield_nrow::Vector{Cint}
   hfield_ncol::Vector{Cint}
   hfield_adr::Vector{Cint}
   hfield_data::Vector{Cfloat}

   tex_type::Vector{Cint}
   tex_height::Vector{Cint}
   tex_width::Vector{Cint}
   tex_adr::Vector{Cint}
   tex_rgb::Vector{mjtByte}

   mat_texid::Vector{Cint}
   mat_texuniform::Vector{mjtByte}
   mat_texrepeat::Matrix{Cfloat}
   mat_emission::Vector{Cfloat}
   mat_specular::Vector{Cfloat}
   mat_shininess::Vector{Cfloat}
   mat_reflectance::Vector{Cfloat}
   mat_rgba::Matrix{Cfloat}

   pair_dim::Vector{Cint}
   pair_geom1::Vector{Cint}
   pair_geom2::Vector{Cint}
   pair_signature::Vector{Cint}
   pair_solref::Matrix{mjtNum}
   pair_solimp::Matrix{mjtNum}
   pair_margin::Vector{mjtNum}
   pair_gap::Vector{mjtNum}
   pair_friction::Matrix{mjtNum}

   exclude_signature::Vector{Cint}

   eq_type::Vector{Cint}
   eq_obj1id::Vector{Cint}
   eq_obj2id::Vector{Cint}
   eq_active::Vector{mjtByte}
   eq_solref::Matrix{mjtNum}
   eq_solimp::Matrix{mjtNum}
   eq_data::Matrix{mjtNum}

   tendon_adr::Vector{Cint}
   tendon_num::Vector{Cint}
   tendon_matid::Vector{Cint}
   tendon_group::Vector{Cint}
   tendon_limited::Vector{mjtByte}
   tendon_width::Vector{mjtNum}
   tendon_solref_lim::Matrix{mjtNum}
   tendon_solimp_lim::Matrix{mjtNum}
   tendon_solref_fri::Matrix{mjtNum}
   tendon_solimp_fri::Matrix{mjtNum}
   tendon_range::Matrix{mjtNum}
   tendon_margin::Vector{mjtNum}
   tendon_stiffness::Vector{mjtNum}
   tendon_damping::Vector{mjtNum}
   tendon_frictionloss::Vector{mjtNum}
   tendon_lengthspring::Vector{mjtNum}
   tendon_length0::Vector{mjtNum}
   tendon_invweight0::Vector{mjtNum}
   tendon_user::Matrix{mjtNum}
   tendon_rgba::Matrix{Cfloat}

   wrap_type::Vector{Cint}
   wrap_objid::Vector{Cint}
   wrap_prm::Vector{mjtNum}

   actuator_trntype::Vector{Cint}
   actuator_dyntype::Vector{Cint}
   actuator_gaintype::Vector{Cint}
   actuator_biastype::Vector{Cint}
   actuator_trnid::Matrix{Cint}
   actuator_group::Vector{Cint}
   actuator_ctrllimited::Vector{mjtByte}
   actuator_forcelimited::Vector{mjtByte}
   actuator_dynprm::Matrix{mjtNum}
   actuator_gainprm::Matrix{mjtNum}
   actuator_biasprm::Matrix{mjtNum}
   actuator_ctrlrange::Matrix{mjtNum}
   actuator_forcerange::Matrix{mjtNum}
   actuator_gear::Matrix{mjtNum}
   actuator_cranklength::Vector{mjtNum}
   actuator_acc0::Vector{mjtNum}
   actuator_length0::Vector{mjtNum}
   actuator_lengthrange::Matrix{mjtNum}
   actuator_user::Matrix{mjtNum}

   sensor_type::Vector{Cint}
   sensor_datatype::Vector{Cint}
   sensor_needstage::Vector{Cint}
   sensor_objtype::Vector{Cint}
   sensor_objid::Vector{Cint}
   sensor_dim::Vector{Cint}
   sensor_adr::Vector{Cint}
   sensor_cutoff::Vector{mjtNum}
   sensor_noise::Vector{mjtNum}
   sensor_user::Matrix{mjtNum}

   numeric_adr::Vector{Cint}
   numeric_size::Vector{Cint}
   numeric_data::Vector{mjtNum}

   text_adr::Vector{Cint}
   text_size::Vector{Cint}
   text_data::Vector{UInt8}

   tuple_adr::Vector{Cint}
   tuple_size::Vector{Cint}
   tuple_objtype::Vector{Cint}
   tuple_objid::Vector{Cint}
   tuple_objprm::Vector{mjtNum}

   key_time::Vector{mjtNum}
   key_qpos::Matrix{mjtNum}
   key_qvel::Matrix{mjtNum}
   key_act::Matrix{mjtNum}

   name_bodyadr::Vector{Cint}
   name_jntadr::Vector{Cint}
   name_geomadr::Vector{Cint}
   name_siteadr::Vector{Cint}
   name_camadr::Vector{Cint}
   name_lightadr::Vector{Cint}
   name_meshadr::Vector{Cint}
   name_skinadr::Vector{Cint}
   name_hfieldadr::Vector{Cint}
   name_texadr::Vector{Cint}
   name_matadr::Vector{Cint}
   name_pairadr::Vector{Cint}
   name_excludeadr::Vector{Cint}
   name_eqadr::Vector{Cint}
   name_tendonadr::Vector{Cint}
   name_actuatoradr::Vector{Cint}
   name_sensoradr::Vector{Cint}
   name_numericadr::Vector{Cint}
   name_textadr::Vector{Cint}
   name_tupleadr::Vector{Cint}
   name_keyadr::Vector{Cint}
   names::Vector{UInt8}
end

struct jlData
   d::Ref{mjData} # point to c struct

   stack::Vector{mjtNum}

   qpos::Vector{mjtNum}
   qvel::Vector{mjtNum}
   act::Vector{mjtNum}
   qacc_warmstart::Vector{mjtNum}
   ctrl::Vector{mjtNum}
   qfrc_applied::Vector{mjtNum}
   xfrc_applied::Matrix{mjtNum}
   qacc::Vector{mjtNum}
   act_dot::Vector{mjtNum}
   mocap_pos::Matrix{mjtNum}
   mocap_quat::Matrix{mjtNum}
   userdata::Vector{mjtNum}
   sensordata::Vector{mjtNum}

   xpos::Matrix{mjtNum}
   xquat::Matrix{mjtNum}
   xmat::Matrix{mjtNum}
   xipos::Matrix{mjtNum}
   ximat::Matrix{mjtNum}
   xanchor::Matrix{mjtNum}
   xaxis::Matrix{mjtNum}
   geom_xpos::Matrix{mjtNum}
   geom_xmat::Matrix{mjtNum}
   site_xpos::Matrix{mjtNum}
   site_xmat::Matrix{mjtNum}
   cam_xpos::Matrix{mjtNum}
   cam_xmat::Matrix{mjtNum}
   light_xpos::Matrix{mjtNum}
   light_xdir::Matrix{mjtNum}

   subtree_com::Matrix{mjtNum}
   cdof::Matrix{mjtNum}
   cinert::Matrix{mjtNum}
   ten_wrapadr::Vector{Cint}
   ten_wrapnum::Vector{Cint}
   ten_J_rownnz::Vector{Cint}
   ten_J_rowadr::Vector{Cint}
   ten_J_colind::Matrix{Cint}
   ten_length::Vector{mjtNum}
   ten_J::Matrix{mjtNum}
   wrap_obj::Vector{Cint}
   wrap_xpos::Matrix{mjtNum}
   actuator_length::Vector{mjtNum}
   actuator_moment::Matrix{mjtNum}

   crb::Matrix{mjtNum}
   qM::Vector{mjtNum}
   qLD::Vector{mjtNum}
   qLDiagInv::Vector{mjtNum}
   qLDiagSqrtInv::Vector{mjtNum}

   contact::Vector{mjContact}

   efc_type::Vector{Cint}
   efc_id::Vector{Cint}
   efc_J_rownnz::Vector{Cint}
   efc_J_rowadr::Vector{Cint}
   efc_J_rowsuper::Vector{Cint}
   efc_J_colind::Matrix{Cint}
   efc_JT_rownnz::Vector{Cint}
   efc_JT_rowadr::Vector{Cint}
   efc_JT_rowsuper::Vector{Cint}
   efc_JT_colind::Matrix{Cint}
   efc_J::Matrix{mjtNum}
   efc_JT::Matrix{mjtNum}
   efc_pos::Vector{mjtNum}
   efc_margin::Vector{mjtNum}
   efc_frictionloss::Vector{mjtNum}
   efc_diagApprox::Vector{mjtNum}
   efc_KBIP::Matrix{mjtNum}
   efc_D::Vector{mjtNum}
   efc_R::Vector{mjtNum}

   efc_AR_rownnz::Vector{Cint}
   efc_AR_rowadr::Vector{Cint}
   efc_AR_colind::Matrix{Cint}
   efc_AR::Matrix{mjtNum}

   ten_velocity::Vector{mjtNum}
   actuator_velocity::Vector{mjtNum}
   cvel::Matrix{mjtNum}
   cdof_dot::Matrix{mjtNum}
   qfrc_bias::Vector{mjtNum}
   qfrc_passive::Vector{mjtNum}
   efc_vel::Vector{mjtNum}
   efc_aref::Vector{mjtNum}
   subtree_linvel::Matrix{mjtNum}
   subtree_angmom::Matrix{mjtNum}
   actuator_force::Vector{mjtNum}
   qfrc_actuator::Vector{mjtNum}
   qfrc_unc::Vector{mjtNum}
   qacc_unc::Vector{mjtNum}

   efc_b::Vector{mjtNum}
   efc_force::Vector{mjtNum}
   efc_state::Vector{Cint}
   qfrc_constraint::Vector{mjtNum}

   qfrc_inverse::Vector{mjtNum}

   cacc::Matrix{mjtNum}
   cfrc_int::Matrix{mjtNum}
   cfrc_ext::Matrix{mjtNum}
end

# added stack field manually
# manually did second terms in the parantheses
function getdatasize(m::mjModel, d::mjData)
   return Dict(
               :stack=>(d.nstack,1),

               :qpos=>(m.nq,1),
               :qvel=>(m.nv,1),
               :act=>(m.na,1),
               :qacc_warmstart=>(m.nv,1),
               :ctrl=>(m.nu,1),
               :qfrc_applied=>(m.nv,1),
               :xfrc_applied=>(m.nbody,6),
               :qacc=>(m.nv,1),
               :act_dot=>(m.na,1),
               :mocap_pos=>(m.nmocap,3),
               :mocap_quat=>(m.nmocap,4),
               :userdata=>(m.nuserdata,1),
               :sensordata=>(m.nsensordata,1),

               :xpos=>(m.nbody,3),
               :xquat=>(m.nbody,4),
               :xmat=>(m.nbody,9),
               :xipos=>(m.nbody,3),
               :ximat=>(m.nbody,9),
               :xanchor=>(m.njnt,3),
               :xaxis=>(m.njnt,3),
               :geom_xpos=>(m.ngeom,3),
               :geom_xmat=>(m.ngeom,9),
               :site_xpos=>(m.nsite,3),
               :site_xmat=>(m.nsite,9),
               :cam_xpos=>(m.ncam,3),
               :cam_xmat=>(m.ncam,9),
               :light_xpos=>(m.nlight,3),
               :light_xdir=>(m.nlight,3),

               :subtree_com=>(m.nbody,3),
               :cdof=>(m.nv,6),
               :cinert=>(m.nbody,10),
               :ten_wrapadr=>(m.ntendon,1),
               :ten_wrapnum=>(m.ntendon,1),
               :ten_J_rownnz=>(m.ntendon,1),
               :ten_J_rowadr=>(m.ntendon,1),
               :ten_J_colind=>(m.ntendon,m.nv),
               :ten_length=>(m.ntendon,1),
               :ten_J=>(m.ntendon,m.nv),
               :wrap_obj=>(m.nwrap*2,1),
               :wrap_xpos=>(m.nwrap*2,3),
               :actuator_length=>(m.nu,1),
               :actuator_moment=>(m.nu,m.nv),

               :crb=>(m.nbody,10),
               :qM=>(m.nM,1),
               :qLD=>(m.nM,1),
               :qLDiagInv=>(m.nv,1),
               :qLDiagSqrtInv=>(m.nv,1),

               :contact=>(m.nconmax,1),

               :efc_type=>(m.njmax,1),
               :efc_id=>(m.njmax,1),
               :efc_J_rownnz=>(m.njmax,1),
               :efc_J_rowadr=>(m.njmax,1),
               :efc_J_rowsuper=>(m.njmax,1),
               :efc_J_colind=>(m.njmax,m.nv),
               :efc_JT_rownnz=>(m.nv,1),
               :efc_JT_rowadr=>(m.nv,1),
               :efc_JT_rowsuper=>(m.nv,1),
               :efc_JT_colind=>(m.nv,m.njmax),
               :efc_J=>(m.njmax,m.nv),
               :efc_JT=>(m.nv,m.njmax),
               :efc_pos=>(m.njmax,1),
               :efc_margin=>(m.njmax,1),
               :efc_frictionloss=>(m.njmax,1),
               :efc_diagApprox=>(m.njmax,1),
               :efc_KBIP=>(m.njmax,4),
               :efc_D=>(m.njmax,1),
               :efc_R=>(m.njmax,1),

               :efc_AR_rownnz=>(m.njmax,1),
               :efc_AR_rowadr=>(m.njmax,1),
               :efc_AR_colind=>(m.njmax,m.njmax),
               :efc_AR=>(m.njmax,m.njmax),

               :ten_velocity=>(m.ntendon,1),
               :actuator_velocity=>(m.nu,1),
               :cvel=>(m.nbody,6),
               :cdof_dot=>(m.nv,6),
               :qfrc_bias=>(m.nv,1),
               :qfrc_passive=>(m.nv,1),
               :efc_vel=>(m.njmax,1),
               :efc_aref=>(m.njmax,1),
               :subtree_linvel=>(m.nbody,3),
               :subtree_angmom=>(m.nbody,3),
               :actuator_force=>(m.nu,1),
               :qfrc_actuator=>(m.nv,1),
               :qfrc_unc=>(m.nv,1),
               :qacc_unc=>(m.nv,1),

               :efc_b=>(m.njmax,1),
               :efc_force=>(m.njmax,1),
               :efc_state=>(m.njmax,1),
               :qfrc_constraint=>(m.nv,1),

               :qfrc_inverse=>(m.nv,1),

               :cacc=>(m.nbody,6),
               :cfrc_int=>(m.nbody,6),
               :cfrc_ext=>(m.nbody,6)
              )
end

function getmodelsize(m::mjModel)
   return Dict(
               :qpos0=>(m.nq,1),
               :qpos_spring=>(m.nq,1),

               :body_parentid=>(m.nbody,1),
               :body_rootid=>(m.nbody,1),
               :body_weldid=>(m.nbody,1),
               :body_mocapid=>(m.nbody,1),
               :body_jntnum=>(m.nbody,1),
               :body_jntadr=>(m.nbody,1),
               :body_dofnum=>(m.nbody,1),
               :body_dofadr=>(m.nbody,1),
               :body_geomnum=>(m.nbody,1),
               :body_geomadr=>(m.nbody,1),
               :body_simple=>(m.nbody,1),
               :body_sameframe=>(m.nbody,1),
               :body_pos=>(m.nbody,3),
               :body_quat=>(m.nbody,4),
               :body_ipos=>(m.nbody,3),
               :body_iquat=>(m.nbody,4),
               :body_mass=>(m.nbody,1),
               :body_subtreemass=>(m.nbody,1),
               :body_inertia=>(m.nbody,3),
               :body_invweight0=>(m.nbody,2),
               :body_user=>(m.nbody,m.nuser_body),

               :jnt_type=>(m.njnt,1),
               :jnt_qposadr=>(m.njnt,1),
               :jnt_dofadr=>(m.njnt,1),
               :jnt_bodyid=>(m.njnt,1),
               :jnt_group=>(m.njnt,1),
               :jnt_limited=>(m.njnt,1),
               :jnt_solref=>(m.njnt,NREF),
               :jnt_solimp=>(m.njnt,NIMP),
               :jnt_pos=>(m.njnt,3),
               :jnt_axis=>(m.njnt,3),
               :jnt_stiffness=>(m.njnt,1),
               :jnt_range=>(m.njnt,2),
               :jnt_margin=>(m.njnt,1),
               :jnt_user=>(m.njnt,m.nuser_jnt),

               :dof_bodyid=>(m.nv,1),
               :dof_jntid=>(m.nv,1),
               :dof_parentid=>(m.nv,1),
               :dof_Madr=>(m.nv,1),
               :dof_simplenum=>(m.nv,1),
               :dof_solref=>(m.nv,NREF),
               :dof_solimp=>(m.nv,NIMP),
               :dof_frictionloss=>(m.nv,1),
               :dof_armature=>(m.nv,1),
               :dof_damping=>(m.nv,1),
               :dof_invweight0=>(m.nv,1),
               :dof_M0=>(m.nv,1),

               :geom_type=>(m.ngeom,1),
               :geom_contype=>(m.ngeom,1),
               :geom_conaffinity=>(m.ngeom,1),
               :geom_condim=>(m.ngeom,1),
               :geom_bodyid=>(m.ngeom,1),
               :geom_dataid=>(m.ngeom,1),
               :geom_matid=>(m.ngeom,1),
               :geom_group=>(m.ngeom,1),
               :geom_priority=>(m.ngeom,1),
               :geom_sameframe=>(m.ngeom,1),
               :geom_solmix=>(m.ngeom,1),
               :geom_solref=>(m.ngeom,NREF),
               :geom_solimp=>(m.ngeom,NIMP),
               :geom_size=>(m.ngeom,3),
               :geom_rbound=>(m.ngeom,1),
               :geom_pos=>(m.ngeom,3),
               :geom_quat=>(m.ngeom,4),
               :geom_friction=>(m.ngeom,3),
               :geom_margin=>(m.ngeom,1),
               :geom_gap=>(m.ngeom,1),
               :geom_user=>(m.ngeom,m.nuser_geom),
               :geom_rgba=>(m.ngeom,4),

               :site_type=>(m.nsite,1),
               :site_bodyid=>(m.nsite,1),
               :site_matid=>(m.nsite,1),
               :site_group=>(m.nsite,1),
               :site_sameframe=>(m.nsite,1),
               :site_size=>(m.nsite,3),
               :site_pos=>(m.nsite,3),
               :site_quat=>(m.nsite,4),
               :site_user=>(m.nsite,m.nuser_site),
               :site_rgba=>(m.nsite,4),

               :cam_mode=>(m.ncam,1),
               :cam_bodyid=>(m.ncam,1),
               :cam_targetbodyid=>(m.ncam,1),
               :cam_pos=>(m.ncam,3),
               :cam_quat=>(m.ncam,4),
               :cam_poscom0=>(m.ncam,3),
               :cam_pos0=>(m.ncam,3),
               :cam_mat0=>(m.ncam,9),
               :cam_fovy=>(m.ncam,1),
               :cam_ipd=>(m.ncam,1),
               :cam_user=>(m.ncam,m.nuser_cam),

               :light_mode=>(m.nlight,1),
               :light_bodyid=>(m.nlight,1),
               :light_targetbodyid=>(m.nlight,1),
               :light_directional=>(m.nlight,1),
               :light_castshadow=>(m.nlight,1),
               :light_active=>(m.nlight,1),
               :light_pos=>(m.nlight,3),
               :light_dir=>(m.nlight,3),
               :light_poscom0=>(m.nlight,3),
               :light_pos0=>(m.nlight,3),
               :light_dir0=>(m.nlight,3),
               :light_attenuation=>(m.nlight,3),
               :light_cutoff=>(m.nlight,1),
               :light_exponent=>(m.nlight,1),
               :light_ambient=>(m.nlight,3),
               :light_diffuse=>(m.nlight,3),
               :light_specular=>(m.nlight,3),

               :mesh_vertadr=>(m.nmesh,1),
               :mesh_vertnum=>(m.nmesh,1),
               :mesh_texcoordadr=>(m.nmesh,1),
               :mesh_faceadr=>(m.nmesh,1),
               :mesh_facenum=>(m.nmesh,1),
               :mesh_graphadr=>(m.nmesh,1),
               :mesh_vert=>(m.nmeshvert,3),
               :mesh_normal=>(m.nmeshvert,3),
               :mesh_texcoord=>(m.nmeshtexvert,2),
               :mesh_face=>(m.nmeshface,3),
               :mesh_graph=>(m.nmeshgraph,1),

               :skin_matid=>(m.nskin,1),
               :skin_rgba=>(m.nskin,4),
               :skin_inflate=>(m.nskin,1),
               :skin_vertadr=>(m.nskin,1),
               :skin_vertnum=>(m.nskin,1),
               :skin_texcoordadr=>(m.nskin,1),
               :skin_faceadr=>(m.nskin,1),
               :skin_facenum=>(m.nskin,1),
               :skin_boneadr=>(m.nskin,1),
               :skin_bonenum=>(m.nskin,1),
               :skin_vert=>(m.nskinvert,3),
               :skin_texcoord=>(m.nskintexvert,2),
               :skin_face=>(m.nskinface,3),
               :skin_bonevertadr=>(m.nskinbone,1),
               :skin_bonevertnum=>(m.nskinbone,1),
               :skin_bonebindpos=>(m.nskinbone,3),
               :skin_bonebindquat=>(m.nskinbone,4),
               :skin_bonebodyid=>(m.nskinbone,1),
               :skin_bonevertid=>(m.nskinbonevert,1),
               :skin_bonevertweight=>(m.nskinbonevert,1),

               :hfield_size=>(m.nhfield,4),
               :hfield_nrow=>(m.nhfield,1),
               :hfield_ncol=>(m.nhfield,1),
               :hfield_adr=>(m.nhfield,1),
               :hfield_data=>(m.nhfielddata,1),

               :tex_type=>(m.ntex,1),
               :tex_height=>(m.ntex,1),
               :tex_width=>(m.ntex,1),
               :tex_adr=>(m.ntex,1),
               :tex_rgb=>(m.ntexdata,1),

               :mat_texid=>(m.nmat,1),
               :mat_texuniform=>(m.nmat,1),
               :mat_texrepeat=>(m.nmat,2),
               :mat_emission=>(m.nmat,1),
               :mat_specular=>(m.nmat,1),
               :mat_shininess=>(m.nmat,1),
               :mat_reflectance=>(m.nmat,1),
               :mat_rgba=>(m.nmat,4),

               :pair_dim=>(m.npair,1),
               :pair_geom1=>(m.npair,1),
               :pair_geom2=>(m.npair,1),
               :pair_signature=>(m.npair,1),
               :pair_solref=>(m.npair,NREF),
               :pair_solimp=>(m.npair,NIMP),
               :pair_margin=>(m.npair,1),
               :pair_gap=>(m.npair,1),
               :pair_friction=>(m.npair,5),

               :exclude_signature=>(m.nexclude,1),

               :eq_type=>(m.neq,1),
               :eq_obj1id=>(m.neq,1),
               :eq_obj2id=>(m.neq,1),
               :eq_active=>(m.neq,1),
               :eq_solref=>(m.neq,NREF),
               :eq_solimp=>(m.neq,NIMP),
               :eq_data=>(m.neq,NEQDATA),

               :tendon_adr=>(m.ntendon,1),
               :tendon_num=>(m.ntendon,1),
               :tendon_matid=>(m.ntendon,1),
               :tendon_group=>(m.ntendon,1),
               :tendon_limited=>(m.ntendon,1),
               :tendon_width=>(m.ntendon,1),
               :tendon_solref_lim=>(m.ntendon,NREF),
               :tendon_solimp_lim=>(m.ntendon,NIMP),
               :tendon_solref_fri=>(m.ntendon,NREF),
               :tendon_solimp_fri=>(m.ntendon,NIMP),
               :tendon_range=>(m.ntendon,2),
               :tendon_margin=>(m.ntendon,1),
               :tendon_stiffness=>(m.ntendon,1),
               :tendon_damping=>(m.ntendon,1),
               :tendon_frictionloss=>(m.ntendon,1),
               :tendon_lengthspring=>(m.ntendon,1),
               :tendon_length0=>(m.ntendon,1),
               :tendon_invweight0=>(m.ntendon,1),
               :tendon_user=>(m.ntendon,m.nuser_tendon),
               :tendon_rgba=>(m.ntendon,4),

               :wrap_type=>(m.nwrap,1),
               :wrap_objid=>(m.nwrap,1),
               :wrap_prm=>(m.nwrap,1),

               :actuator_trntype=>(m.nu,1),
               :actuator_dyntype=>(m.nu,1),
               :actuator_gaintype=>(m.nu,1),
               :actuator_biastype=>(m.nu,1),
               :actuator_trnid=>(m.nu,2),
               :actuator_group=>(m.nu,1),
               :actuator_ctrllimited=>(m.nu,1),
               :actuator_forcelimited=>(m.nu,1),
               :actuator_dynprm=>(m.nu,NDYN),
               :actuator_gainprm=>(m.nu,NGAIN),
               :actuator_biasprm=>(m.nu,NBIAS),
               :actuator_ctrlrange=>(m.nu,2),
               :actuator_forcerange=>(m.nu,2),
               :actuator_gear=>(m.nu,6),
               :actuator_cranklength=>(m.nu,1),
               :actuator_acc0=>(m.nu,1),
               :actuator_length0=>(m.nu,1),
               :actuator_lengthrange=>(m.nu,2),
               :actuator_user=>(m.nu,m.nuser_actuator),

               :sensor_type=>(m.nsensor,1),
               :sensor_datatype=>(m.nsensor,1),
               :sensor_needstage=>(m.nsensor,1),
               :sensor_objtype=>(m.nsensor,1),
               :sensor_objid=>(m.nsensor,1),
               :sensor_dim=>(m.nsensor,1),
               :sensor_adr=>(m.nsensor,1),
               :sensor_cutoff=>(m.nsensor,1),
               :sensor_noise=>(m.nsensor,1),
               :sensor_user=>(m.nsensor,m.nuser_sensor),

               :numeric_adr=>(m.nnumeric,1),
               :numeric_size=>(m.nnumeric,1),
               :numeric_data=>(m.nnumericdata,1),

               :text_adr=>(m.ntext,1),
               :text_size=>(m.ntext,1),
               :text_data=>(m.ntextdata,1),

               :tuple_adr=>(m.ntuple,1),
               :tuple_size=>(m.ntuple,1),
               :tuple_objtype=>(m.ntupledata,1),
               :tuple_objid=>(m.ntupledata,1),
               :tuple_objprm=>(m.ntupledata,1),

               :key_time=>(m.nkey,1),
               :key_qpos=>(m.nkey,m.nq),
               :key_qvel=>(m.nkey,m.nv),
               :key_act=>(m.nkey,m.na),

               :name_bodyadr=>(m.nbody,1),
               :name_jntadr=>(m.njnt,1),
               :name_geomadr=>(m.ngeom,1),
               :name_siteadr=>(m.nsite,1),
               :name_camadr=>(m.ncam,1),
               :name_lightadr=>(m.nlight,1),
               :name_meshadr=>(m.nmesh,1),
               :name_skinadr=>(m.nskin,1),
               :name_hfieldadr=>(m.nhfield,1),
               :name_texadr=>(m.ntex,1),
               :name_matadr=>(m.nmat,1),
               :name_pairadr=>(m.npair,1),
               :name_excludeadr=>(m.nexclude,1),
               :name_eqadr=>(m.neq,1),
               :name_tendonadr=>(m.ntendon,1),
               :name_actuatoradr=>(m.nu,1),
               :name_sensoradr=>(m.nsensor,1),
               :name_numericadr=>(m.nnumeric,1),
               :name_textadr=>(m.ntext,1),
               :name_tupleadr=>(m.ntuple,1),
               :name_keyadr=>(m.nkey,1),
               :names=>(m.nnames,1)
              )
end

