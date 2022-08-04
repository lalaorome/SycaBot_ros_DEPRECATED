import sys
import time
import numpy as np
import scipy.linalg
from casadi import SX, vertcat, sin, cos
import matplotlib.pyplot as plt
import math

from jetbot.jb_utils import *
from jetbot.jb_action_Ctrller import CtrllerActionServer

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from interfaces.action import Control
from geometry_msgs.msg import Pose2D

from acados_template import AcadosOcp, AcadosOcpSolver
from acados_template import AcadosModel


class MPC(CtrllerActionServer):
    # https://blog.actorsfit.com/a?ID=01550-fd216bc6-e1d5-4d16-b901-143d2b19c430
    def __init__(self):
        super().__init__('MPC')
        self.declare_parameter('Q', [1.,0.,0.,0.,1.,0.,0.,0.,0.2])
        self.declare_parameter('R', [0.5,0.,0.,0.5])
        self.declare_parameter('M', 10.)
        self.declare_parameter('radius_safeset', 4.)
        self.declare_parameter('timesteps', 20)
        self.declare_parameter('horizon', 2.)

        self.Q=self.get_parameter('Q').value
        self.R_mat=self.get_parameter('R').value
        self.M=self.get_parameter('M').value
        self.sr = self.get_parameter('radius_safeset').value
        self.N = self.get_parameter('timesteps').value
        self.Tf = self.get_parameter('horizon').value

        self.viz_pathref_pub = self.create_publisher(Pose2D, f'/SycaBot_W{self.id}/visualisation', 10)
        self.ocp_solver = self.config_ocp()

    def control_cb(self, goal_handle):
        result = Control.Result()

        self.wait4pose()

        t_sim = 0
        [state_plot, input_plot] = self.get_reference(0,0.1,200)
        
        # set cost
        ocp_solver = self.ocp_solver
        
        Ts_MPC = self.Tf / self.N
        t_init = time.time()
        x0 = self.rob_state
        while t_sim < 20. :

            t_loop = time.time()
            # update initial condition
            previous_x0 = x0
            x0 = self.rob_state
            x0[2] = np.arctan2(np.sin(x0[2] - previous_x0[2]),np.cos(x0[2] - previous_x0[2])) + previous_x0[2]

            ocp_solver.set(0, "lbx", x0)
            ocp_solver.set(0, "ubx", x0)

            # reference
            [state_ref, input_ref] = self.get_reference(t_sim, Ts_MPC, self.N)

            for k in range(self.N):
                if k == 0:
                    theta_ref_k = np.arctan2(np.sin(state_ref[2,k] - x0[2]),np.cos(state_ref[2,k] - x0[2])) + x0[2]
                else:
                    theta_ref_k = np.arctan2(np.sin(state_ref[2,k] - state_ref[2,k - 1]),np.cos(state_ref[2,k] - state_ref[2,k - 1])) + state_ref[2,k - 1]
                ocp_solver.set(k, "yref", np.array([state_ref[0,k], state_ref[1,k], theta_ref_k, input_ref[0,k], input_ref[1,k]]))
                ocp_solver.set(k, "p", np.array([state_ref[0,k], state_ref[1,k]]))
            theta_ref_N = np.arctan2(np.sin(state_ref[2,self.N] - state_ref[2,self.N - 1]),np.cos(state_ref[2,self.N] - state_ref[2,self.N - 1])) + state_ref[2,self.N - 1]
            ocp_solver.set(self.N, "yref",np.array([state_ref[0,self.N], state_ref[1,self.N], theta_ref_N]))
            ocp_solver.set(self.N, "p", np.array([state_ref[0,self.N], state_ref[1,self.N]]))
            
            # feedback rti_phase
            status = ocp_solver.solve()
            solver_time = time.time()
            self.get_logger().info(f"Solver time is {solver_time-t_loop}s")

            if status != 0:
                raise Exception(f'acados returned status {status}.')

            
            # get solution
            # x0 = ocp_solver.get(0, "x")
            u0 = ocp_solver.get(0, "u")
            self.get_logger().info(f"Control input : {u0}")
            Vr,Vl = self.velocities2wheelinput(u0[0],u0[1])
            self.sendVelCmd(Vr,Vl)
            path_ref = Pose2D()
            path_ref.x = state_ref[0,0]
            path_ref.y = state_ref[1,0]
            # self.viz_pathref_pub.publish(path_ref)
            other_time = time.time()
            self.get_logger().info(f"Other time is {other_time-solver_time}s")
            time.sleep(max(Ts_MPC - solver_time - other_time,0))
            
            t_sim = time.time() - t_init
            # upred = np.zeros((nu,N))
            # xpred  = np.zeros((nx,N + 1))
            # for k in range(N):
            #     upred[:,k] = ocp_solver.get(k,"u")
            #     xpred[:,k] = ocp_solver.get(k,"x")
            # xpred[:,N] = ocp_solver.get(N,"x")
            
            # ur_ref = 1.0 / (self.f_coefs[0] * self.R) * input_ref[0,:] + self.L / (2 * self.f_coefs[0] * self.R) * input_ref[1,:]
            # ul_ref = 1.0 / (self.f_coefs[1] * self.R) * input_ref[0,:] - self.L / (2 * self.f_coefs[1] * self.R) * input_ref[1,:]
            # ur_pred = 1.0 / (self.f_coefs[0] * self.R) * upred[0,:] + self.L / (2 * self.f_coefs[0] * self.R) * upred[1,:]
            # ul_pred = 1.0 / (self.f_coefs[1] * self.R) * upred[0,:] - self.L / (2 * self.f_coefs[1] * self.R) * upred[1,:]

        self.stop()
        time.sleep(self.Ts)
        goal_handle.succeed()
        result.success = True
        return result
    
    def export_unicycle_ode_model_with_LocalConstraints(self):
        model_name = 'unicycle_ode'
        # set up states & controls
        x_pos = SX.sym('x_pos')
        y_pos = SX.sym('y_pos')
        theta_orient = SX.sym('theta_orient')
        v = SX.sym('v')
        omega = SX.sym('omega')
        # xdot
        x_pos_dot      = SX.sym('x_pos_dot')
        y_pos_dot      = SX.sym('y_pos_dot')
        theta_orient_dot   = SX.sym('theta_orient_dot')
        # parameters
        x_ref = SX.sym('x_ref') 
        y_ref = SX.sym('y_ref')
        x = vertcat(x_pos, y_pos, theta_orient)
        u = vertcat(v , omega)
        xdot = vertcat(x_pos_dot, y_pos_dot, theta_orient_dot)
        p = vertcat(x_ref, y_ref)
        # dynamics
        f_expl = vertcat(v * cos(theta_orient), v * sin(theta_orient), omega)
        f_impl = xdot - f_expl
        #nonlinear constraint
        con_h_expr = (x_pos - x_ref) ** 2 + (y_pos - y_ref) ** 2

        model = AcadosModel()
        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        model.con_h_expr = con_h_expr  
        model.con_h_expr_e = con_h_expr  
        model.x = x
        model.xdot = xdot
        model.u = u
        model.p = p  
        model.name = model_name

        return model

    def get_reference(self,t,Ts,N):
        ref_rad = 0.5
        ref_T = 10
        t_vec = np.linspace(t,t + N * Ts, N + 1)
        x_pos_ref = ref_rad * np.cos(2 * math.pi / ref_T * t_vec)
        y_pos_ref = ref_rad * np.sin(2 * math.pi / ref_T * t_vec)
        v_ref = 2 * math.pi * ref_rad / ref_T * np.ones(N)
        omega_ref = 2 * math.pi / ref_T * np.ones(N)
        theta_ref = math.pi / 2 + t_vec * 2 * math.pi / ref_T 
        state_ref = np.vstack((x_pos_ref.reshape(1,N + 1), y_pos_ref.reshape(1,N + 1), theta_ref.reshape(1,N + 1)))
        input_ref = np.vstack((v_ref.reshape(1,N), omega_ref.reshape(1,N)))
        return state_ref, input_ref
    
    def get_Q_R(self):
        Q = np.array(self.Q).reshape(3,3)
        R = np.array(self.R_mat).reshape(2,2)
        return Q,R
    
    def config_ocp(self):

        # create ocp object to formulate the OCP
        ocp = AcadosOcp()
        # set model
        model = self.export_unicycle_ode_model_with_LocalConstraints()
        ocp.model = model

        Q,R = self.get_Q_R()
        nx = ocp.model.x.size()[0]
        nu = ocp.model.u.size()[0]
        ny = nx + nu
        ny_e = nx
        nsh = 1
        W_e = 1 * Q
        W = scipy.linalg.block_diag(Q, R)
        Vx = np.zeros((ny, nx))
        Vx[:nx,:nx] = np.eye(nx)
        Vx_e = np.zeros((ny_e, nx))
        Vx_e[:nx, :nx] = np.eye(nx)
        Vu = np.zeros((ny, nu))
        Vu[nx:,:] = np.eye(2)
        Vr_max, Vl_max = 0.4,0.4
        D = np.array([[1. / (self.f_coefs[0] * self.R), self.L / (2. * self.f_coefs[0] * self.R)], [1. / (self.f_coefs[1] * self.R), -self.L / (2. * self.f_coefs[1] * self.R)]])

        ocp.dims.N = self.N
        ocp.cost.Vx = Vx
        ocp.cost.Vx_e = Vx_e
        ocp.cost.Vu = Vu
        ocp.cost.Vu_0 = Vu
        ocp.cost.W_e = W_e
        ocp.cost.W = W
        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.cost_type_e = 'LINEAR_LS'
        # set intial references
        ocp.cost.yref  = np.zeros(ny)
        ocp.cost.yref_e = np.zeros(ny_e)
        # setting input constraints constraints
        ocp.constraints.D = D
        ocp.constraints.C = np.zeros((2,nx))
        ocp.constraints.lg = np.array([-Vr_max, -Vl_max])
        ocp.constraints.ug = np.array([Vr_max, Vl_max])
        # set soft contraint penatly for local safe set
        ocp.cost.zl = np.array([0.])
        ocp.cost.zl_e = np.array([0.])
        ocp.cost.Zl = np.array([0.])
        ocp.cost.Zl_e = np.array([0.])
        ocp.cost.zu = self.M * np.amax(W_e) * np.ones((nsh,))
        ocp.cost.zu_e = self.M * np.amax(W_e) * np.ones((nsh,))
        ocp.cost.Zu = np.array([0.])
        ocp.cost.Zu_e = np.array([0.])
        ocp.constraints.ush = np.zeros(nsh)
        ocp.constraints.uh = np.array([self.sr ** 2])
        ocp.constraints.ush_e = np.zeros(nsh)
        ocp.constraints.uh_e = np.array([self.sr ** 2])
        ocp.constraints.lsh = np.zeros(nsh)
        ocp.constraints.lh = np.array([0.])
        ocp.constraints.lsh_e = np.zeros(nsh)
        ocp.constraints.lh_e = np.array([0.])
        ocp.constraints.idxsh = np.array([0])
        ocp.constraints.idxsh_e = np.array([0])
        ocp.parameter_values = np.array([0., 0.])
        ocp.constraints.x0 = self.rob_state
        # set options
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM' # FULL_CONDENSING_QPOASES
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI' # SQP_RTI, SQP
        # set prediction horizon
        ocp.solver_options.tf = self.Tf
        ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp.json')
        return ocp_solver


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = MPC()
    executor.add_node(node)
    try :
        executor.spin()
    except Exception as e :
        print(e)
    finally:
        node.stop()
        executor.shutdown()
        node.destroy_node()
    return
    


if __name__ == '__main__':
    main()