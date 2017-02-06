#!/usr/bin/env python

from hrpsys_choreonoid_tutorials.choreonoid_hrpsys_config import *

class JAXON_RED_HrpsysConfigurator(ChoreonoidHrpsysConfiguratorOrg):
    acf = None
    acf_svc = None
    acf_version = None

    def getRTCList (self):
        ##return self.getRTCListUnstable()
        return [
            ['seq', "SequencePlayer"],
            ['sh', "StateHolder"],
            ['fk', "ForwardKinematics"],
            ['tf', "TorqueFilter"],
            ['kf', "KalmanFilter"],
            ['vs', "VirtualForceSensor"],
            ['rmfo', "RemoveForceSensorLinkOffset"],
            ['es', "EmergencyStopper"],
            ['ic', "ImpedanceController"],
            ['abc', "AutoBalancer"],
            ['st', "Stabilizer"],
            ['co', "CollisionDetector"],
            # ['tc', "TorqueController"],
            # ['te', "ThermoEstimator"],
            # ['tl', "ThermoLimiter"],
            ['acf', "AccelerationFilter"],
            ['rfu', "ReferenceForceUpdater"],
            ['hes', "EmergencyStopper"],
            ['el', "SoftErrorLimiter"],
            ['log', "DataLogger"]
            ]
    def connectComps(self):
        ChoreonoidHrpsysConfiguratorOrg.connectComps(self)
        if self.acf:
            #   currently use first acc and rate sensors for kf
            s_acc = filter(lambda s: s.type == 'Acceleration', self.sensors)
            if (len(s_acc) > 0) and self.rh.port(s_acc[0].name) != None:  # check existence of sensor ;; currently original HRP4C.xml has different naming rule of gsensor and gyrometer
                connectPorts(self.rh.port(s_acc[0].name), self.acf.port('accIn'))
            s_rate = filter(lambda s: s.type == 'RateGyro', self.sensors)
            if (len(s_rate) > 0) and self.rh.port(s_rate[0].name) != None:  # check existence of sensor ;; currently original HRP4C.xml has different naming rule of gsensor and gyrometer
                connectPorts(self.rh.port(s_rate[0].name), self.acf.port("rateIn"))
            if self.kf:
                connectPorts(self.kf.port("rpy"), self.acf.port("rpyIn"))
            if self.abc:
                connectPorts(self.abc.port("basePosOut"), self.acf.port("posIn"))
            if self.rh:
                connectPorts(self.rh.port("WAIST"), self.acf.port("transIn"))
            # connectPorts(self.sh.port("qOut"), self.acf.port(""))
    def setupLogger(self, maxlen=15000):
        ChoreonoidHrpsysConfiguratorOrg.setupLogger(self, maxlen)
        self.connectLoggerPort(self.rh, 'LFOOT')
        self.connectLoggerPort(self.rh, 'RFOOT')
        self.connectLoggerPort(self.rh, 'LENDS')
        self.connectLoggerPort(self.rh, 'RENDS')

    def defJointGroups (self):
        rarm_group = ['rarm', ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5', 'RARM_JOINT6', 'RARM_JOINT7']]
        larm_group = ['larm', ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5', 'LARM_JOINT6', 'LARM_JOINT7']]
        rleg_group = ['rleg', ['RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT5']]
        lleg_group = ['lleg', ['LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT5']]
        head_group = ['head', ['HEAD_JOINT0', 'HEAD_JOINT1']]
        torso_group = ['torso', ['CHEST_JOINT0', 'CHEST_JOINT1', 'CHEST_JOINT2']]
        rhand_group = ['rhand', ['RARM_F_JOINT0', 'RARM_F_JOINT1']]
        lhand_group = ['lhand', ['LARM_F_JOINT0', 'LARM_F_JOINT1']]
        range_group = ['range', ['motor_joint']]
        self.Groups = [rarm_group, larm_group, rleg_group, lleg_group, head_group, torso_group, rhand_group, lhand_group, range_group]

    def startABSTIMP (self):
        ### not used on hrpsys
        self.el_svc.setServoErrorLimit("motor_joint",   sys.float_info.max)
        self.el_svc.setServoErrorLimit("RARM_F_JOINT0", sys.float_info.max)
        self.el_svc.setServoErrorLimit("RARM_F_JOINT1", sys.float_info.max)
        self.el_svc.setServoErrorLimit("LARM_F_JOINT0", sys.float_info.max)
        self.el_svc.setServoErrorLimit("LARM_F_JOINT1", sys.float_info.max)
        ###
        self.startAutoBalancer()
        self.ic_svc.startImpedanceController("larm")
        self.ic_svc.startImpedanceController("rarm")
        self.startStabilizer()

if __name__ == '__main__':
    hcf = JAXON_RED_HrpsysConfigurator("JAXON_RED")
    [sys.argv, connect_constraint_force_logger_ports] = hcf.parse_arg_for_connect_ports(sys.argv)
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2], connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
        hcf.startABSTIMP()
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1], connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
        hcf.startABSTIMP()
    else :
        hcf.init(connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
