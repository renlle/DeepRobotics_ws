#include "mapping/fg_manager.h"
#include <fstream>
#include <iomanip>

FactorGraphManager::FactorGraphManager() {
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam_ = new gtsam::ISAM2(parameters);
    gtSAMgraphMade_ = false;
}

FactorGraphManager::~FactorGraphManager() {
    delete isam_;
}

void FactorGraphManager::initNoises(double priorNoiseVar, double odomNoiseVarTrans, double odomNoiseVarRot, 
                                    double loopNoiseScore, double gpsXYNoiseVar, double gpsZNoiseVar) {
    std::lock_guard<std::mutex> lock(mtxPosegraph_);
    
    priorNoiseVar_ = priorNoiseVar;
    odomNoiseVarTrans_ = odomNoiseVarTrans;
    odomNoiseVarRot_ = odomNoiseVarRot;
    loopNoiseScore_ = loopNoiseScore;

    gtsam::Vector priorNoiseVector6(6);
    priorNoiseVector6 << priorNoiseVar, priorNoiseVar, priorNoiseVar, priorNoiseVar, priorNoiseVar, priorNoiseVar;
    priorNoise_ = gtsam::noiseModel::Diagonal::Variances(priorNoiseVector6);

    gtsam::Vector odomNoiseVector6(6);
    odomNoiseVector6 << odomNoiseVarTrans, odomNoiseVarTrans, odomNoiseVarTrans, odomNoiseVarRot, odomNoiseVarRot, odomNoiseVarRot;
    odomNoise_ = gtsam::noiseModel::Diagonal::Variances(odomNoiseVector6);

    gtsam::Vector robustNoiseVector6(6);
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore;
    robustLoopNoise_ = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6)
    );

    gtsam::Vector robustVisualNoiseVector6(6);
    robustVisualNoiseVector6 << loopNoiseScore * 10.0, loopNoiseScore * 10.0, loopNoiseScore * 10.0, 
                                loopNoiseScore * 10.0, loopNoiseScore * 10.0, loopNoiseScore * 10.0;
    robustVisualLoopNoise_ = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
        gtsam::noiseModel::Diagonal::Variances(robustVisualNoiseVector6)
    );

    gtsam::Vector robustNoiseVector3(3);
    robustNoiseVector3 << gpsXYNoiseVar, gpsXYNoiseVar, gpsZNoiseVar;
    robustGPSNoise_ = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector3)
    );
}

bool FactorGraphManager::addPriorFactor(int index, const gtsam::Pose3& pose) {
    std::lock_guard<std::mutex> lock(mtxPosegraph_);
    if (gtSAMgraphMade_) return false;
    gtSAMgraph_.add(gtsam::PriorFactor<gtsam::Pose3>(index, pose, priorNoise_));
    initialEstimate_.insert(index, pose);
    gtSAMgraphMade_ = true;

    auto q = pose.rotation().toQuaternion();
    allFactors_.push_back({
        "PRIOR", index, index,
        pose.translation().x(), pose.translation().y(), pose.translation().z(),
        q.x(), q.y(), q.z(), q.w(),
        priorNoiseVar_, 0.0
    });
    return true;
}

void FactorGraphManager::addOdomFactor(int prev_idx, int curr_idx, const gtsam::Pose3& relative_pose, const gtsam::Pose3& pose_to) {
    std::lock_guard<std::mutex> lock(mtxPosegraph_);
    gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_idx, curr_idx, relative_pose, odomNoise_));
    initialEstimate_.insert(curr_idx, pose_to);

    auto q = relative_pose.rotation().toQuaternion();
    allFactors_.push_back({
        "ODOM", prev_idx, curr_idx,
        relative_pose.translation().x(), relative_pose.translation().y(), relative_pose.translation().z(),
        q.x(), q.y(), q.z(), q.w(),
        odomNoiseVarTrans_, odomNoiseVarRot_
    });
}

void FactorGraphManager::addGPSFactor(int index, const gtsam::Point3& gps_point) {
    std::lock_guard<std::mutex> lock(mtxPosegraph_);
    gtSAMgraph_.add(gtsam::GPSFactor(index, gps_point, robustGPSNoise_));

    allFactors_.push_back({
        "GPS", index, index,
        gps_point.x(), gps_point.y(), gps_point.z(),
        0.0, 0.0, 0.0, 1.0,
        1.0, 0.0
    });
}

void FactorGraphManager::addLoopFactor(int curr_idx, int prev_idx, const gtsam::Pose3& relative_pose) {
    std::lock_guard<std::mutex> lock(mtxPosegraph_);
    gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(curr_idx, prev_idx, relative_pose, robustLoopNoise_));

    auto q = relative_pose.rotation().toQuaternion();
    allFactors_.push_back({
        "LOOP", curr_idx, prev_idx,
        relative_pose.translation().x(), relative_pose.translation().y(), relative_pose.translation().z(),
        q.x(), q.y(), q.z(), q.w(),
        loopNoiseScore_, 0.0
    });
}

void FactorGraphManager::addVisualLoopFactor(int curr_idx, int prev_idx, const gtsam::Pose3& relative_pose) {
    std::lock_guard<std::mutex> lock(mtxPosegraph_);
    gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(curr_idx, prev_idx, relative_pose, robustVisualLoopNoise_));

    auto q = relative_pose.rotation().toQuaternion();
    allFactors_.push_back({
        "VISUAL_LOOP", curr_idx, prev_idx,
        relative_pose.translation().x(), relative_pose.translation().y(), relative_pose.translation().z(),
        q.x(), q.y(), q.z(), q.w(),
        loopNoiseScore_ * 10.0, 0.0
    });
}

void FactorGraphManager::addCrossSessionLoopFactor(int curr_global_idx, int hist_global_idx, const gtsam::Pose3& relative_pose, double score) {
    std::lock_guard<std::mutex> lock(mtxPosegraph_);
    auto q = relative_pose.rotation().toQuaternion();
    // 跨会话闭环仅记录到全量因子库中，留待离线全局图优化联合求解
    allFactors_.push_back({
        "CROSS_LOOP", curr_global_idx, hist_global_idx,
        relative_pose.translation().x(), relative_pose.translation().y(), relative_pose.translation().z(),
        q.x(), q.y(), q.z(), q.w(),
        score, 0.0
    });
}

void FactorGraphManager::addCrossSessionAnchorFactor(int curr_local_idx, const gtsam::Pose3& target_pose_in_local, double icp_score) {
    std::lock_guard<std::mutex> lock(mtxPosegraph_);
    // 以历史冻结帧为绝对真理，赋予其强位姿约束拉正当前帧 (避免柯西核将真实累积大漂移当作离群点严重抑制)
    double trans_var = std::max(0.001, icp_score * 0.05);
    double rot_var = std::max(0.0005, icp_score * 0.02);
    gtsam::Vector noiseVector6(6);
    noiseVector6 << trans_var, trans_var, trans_var, rot_var, rot_var, rot_var;
    auto noiseModel = gtsam::noiseModel::Diagonal::Variances(noiseVector6);
    gtSAMgraph_.add(gtsam::PriorFactor<gtsam::Pose3>(curr_local_idx, target_pose_in_local, noiseModel));
}

void FactorGraphManager::addAnchorFactor(int curr_global_idx, const gtsam::Pose3& anchor_pose) {
    std::lock_guard<std::mutex> lock(mtxPosegraph_);
    auto q = anchor_pose.rotation().toQuaternion();
    allFactors_.push_back({
        "ANCHOR", curr_global_idx, 0,
        anchor_pose.translation().x(), anchor_pose.translation().y(), anchor_pose.translation().z(),
        q.x(), q.y(), q.z(), q.w(),
        priorNoiseVar_, 0.0
    });
}

void FactorGraphManager::saveFactorsToFile(const std::string& filename, bool append, int node_offset) {
    std::lock_guard<std::mutex> lock(mtxPosegraph_);
    std::ios_base::openmode mode = append ? (std::ios::out | std::ios::app) : std::ios::out;
    std::ofstream ofs(filename, mode);
    if (!ofs.is_open()) return;
    ofs << std::fixed << std::setprecision(6);
    for (const auto& f : allFactors_) {
        int f_from = f.from_idx;
        int f_to = f.to_idx;
        if (f.type != "ANCHOR" && f.type != "CROSS_LOOP") {
            f_from += node_offset;
            f_to += node_offset;
        }
        ofs << f.type << " " << f_from << " " << f_to << " "
            << f.x << " " << f.y << " " << f.z << " "
            << f.qx << " " << f.qy << " " << f.qz << " " << f.qw << " "
            << f.noise_val1 << " " << f.noise_val2 << "\n";
    }
    ofs.close();
}

bool FactorGraphManager::runOptimization(int graphUpdateTimes, gtsam::Values& currentEstimate) {
    std::lock_guard<std::mutex> lock(mtxPosegraph_);
    if (!gtSAMgraphMade_) return false;
    isam_->update(gtSAMgraph_, initialEstimate_);
    isam_->update();
    for (int i = graphUpdateTimes; i > 0; --i) {
        isam_->update();
    }
    gtSAMgraph_.resize(0);
    initialEstimate_.clear();
    isamCurrentEstimate_ = isam_->calculateEstimate();
    currentEstimate = isamCurrentEstimate_;
    return true;
}

gtsam::Values FactorGraphManager::getISAMCurrentEstimate() {
    std::lock_guard<std::mutex> lock(mtxPosegraph_);
    return isamCurrentEstimate_;
}

bool FactorGraphManager::isGraphMade() const {
    return gtSAMgraphMade_;
}

std::mutex& FactorGraphManager::getMutex() {
    return mtxPosegraph_;
}
