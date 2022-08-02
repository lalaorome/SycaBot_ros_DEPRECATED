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

from acados_template import AcadosOcp, AcadosOcpSolver
from acados_template import AcadosModel


class MPC(CtrllerActionServer):
    # https://blog.actorsfit.com/a?ID=01550-fd216bc6-e1d5-4d16-b901-143d2b19c430
    def __init__(self):
        super().__init__()

    def control_cb(self, goal_handle):
        result = Control.Result()

        t_sim = 0
        x0 = np.array([0.5, 0.0, math.pi / 2]) + 0.1 * np.random.randn()
        [state_plot, input_plot] = self.get_reference(0,0.1,200)

        WR = 0.03
        WS = 0.11
        r_a = 35.
        l_a = 35.
        r_input_max = 0.4
        r_input_min = -0.4
        l_input_max = 0.4
        l_input_min = -0.4

        # radius of safe set
        sr = 0.25 

        # create ocp object to formulate the OCP
        ocp = AcadosOcp()

        ocp.constraints.x0 = x0

        # set model
        model = self.export_unicycle_ode_model_with_LocalConstraints()
        ocp.model = model

        Tf = 1.0
        nx = model.x.size()[0]
        nu = model.u.size()[0]
        nparam = model.p.size()[0]
        ny = nx + nu
        ny_e = nx
        nsh = 1
        N = 40

        # set dimensions
        ocp.dims.N = N

        # set cost
        Q = np.diag([100.0, 100.0, 1.0])
        R = np.diag([0.1, 0.1])

        W_e = 1 * Q
        W = scipy.linalg.block_diag(Q, R)
        ocp.cost.W_e = W_e
        ocp.cost.W = W

        # print(W)

        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.cost_type_e = 'LINEAR_LS'

        Vx = np.zeros((ny, nx))
        Vx[:nx,:nx] = np.eye(nx)
        ocp.cost.Vx = Vx

        Vx_e = np.zeros((ny_e, nx))
        Vx_e[:nx, :nx] = np.eye(nx)
        ocp.cost.Vx_e = Vx_e

        Vu = np.zeros((ny, nu))
        Vu[nx:,:] = np.eye(2)
        ocp.cost.Vu = Vu
        ocp.cost.Vu_0 = Vu

        # set intial references
        ocp.cost.yref  = np.zeros(ny)
        ocp.cost.yref_e = np.zeros(ny_e)

        # setting input constraints constraints
        D = np.array([[1 / (r_a * WR), WS / (2 * r_a * WR)], [1 / (l_a * WR), -WS / (2 * l_a * WR)]])
        ocp.constraints.D = D
        ocp.constraints.C = np.zeros((2,nx))
        ocp.constraints.lg = np.array([r_input_min, l_input_min])
        ocp.constraints.ug = np.array([r_input_max, l_input_max])

        # set soft contraint penatly for local safe set
        ocp.cost.zl = np.array([0.])
        ocp.cost.zl_e = np.array([0.])
        ocp.cost.Zl = np.array([0.])
        ocp.cost.Zl_e = np.array([0.])
        ocp.cost.zu = 100. * np.amax(W_e) * np.ones((nsh,))
        ocp.cost.zu_e = 100. * np.amax(W_e) * np.ones((nsh,))
        ocp.cost.Zu = np.array([0.])
        ocp.cost.Zu_e = np.array([0.])
        ocp.constraints.ush = np.zeros(nsh)
        ocp.constraints.uh = np.array([sr ** 2])
        ocp.constraints.ush_e = np.zeros(nsh)
        ocp.constraints.uh_e = np.array([sr ** 2])
        ocp.constraints.lsh = np.zeros(nsh)
        ocp.constraints.lh = np.array([0.])
        ocp.constraints.lsh_e = np.zeros(nsh)
        ocp.constraints.lh_e = np.array([0.])
        ocp.constraints.idxsh = np.array([0])
        ocp.constraints.idxsh_e = np.array([0])

        ocp.parameter_values = np.array([0., 0.])


        # set options
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM' # FULL_CONDENSING_QPOASES
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        # ocp.solver_options.print_level = 0
        ocp.solver_options.nlp_solver_type = 'SQP_RTI' # SQP_RTI, SQP

        # set prediction horizon
        ocp.solver_options.tf = Tf

        ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp.json')

        Nsim = 5 *  N

        simX = np.ndarray((nx, Nsim + 1))
        simU = np.ndarray((nu, Nsim))
        simT = np.ndarray((1, Nsim + 1))

        xnext = x0
        t_sim = 0.0
        Ts = Tf / N

        simT[0,0] = t_sim
        for j in range(nx):
            simX[j,0] = xnext[j]

        for i in range(Nsim):
            print("Time ",t_sim)
            # reference
            [state_ref, input_ref] = self.get_reference(t_sim, Ts, N)

            for k in range(N):
                ocp_solver.set(k, "yref", np.array([state_ref[0,k], state_ref[1,k], state_ref[2,k], input_ref[0,k], input_ref[1,k]]))
                ocp_solver.set(k, "p", np.array([state_ref[0,k], state_ref[1,k]]))
            ocp_solver.set(N, "yref",np.array([state_ref[0,N], state_ref[1,N], state_ref[2,N]]))
            ocp_solver.set(N, "p", np.array([state_ref[0,N], state_ref[1,N]]))
            t = time.time()
            
            # preparation rti_phase
            ocp_solver.options_set('rti_phase', 1)
            status = ocp_solver.solve()

            phase1_time = time.time() - t
            print("Phase 1 time time is",phase1_time,"s")

            # update initial condition
            x0 = self.state

            ocp_solver.set(0, "lbx", x0)
            ocp_solver.set(0, "ubx", x0)
            
            t = time.time()

            # feedback rti_phase
            ocp_solver.options_set('rti_phase', 2)
            status = ocp_solver.solve()

            phase2_time = time.time() - t
            print("Phase 2 time time is",phase2_time,"s")

            if status != 0:
                raise Exception(f'acados returned status {status}.')

            
            # get solution
            # x0 = ocp_solver.get(0, "x")
            u0 = ocp_solver.get(0, "u")

            for j in range(nu):
                simU[j,i] = u0[j]

            # upred = np.zeros((nu,N))
            # xpred  = np.zeros((nx,N + 1))
            # for k in range(N):
            #     upred[:,k] = ocp_solver.get(k,"u")
            #     xpred[:,k] = ocp_solver.get(k,"x")
            # xpred[:,N] = ocp_solver.get(N,"x")
            
            # ur_ref = 1.0 / (r_a * WR) * input_ref[0,:] + WS / (2 * r_a * WR) * input_ref[1,:]
            # ul_ref = 1.0 / (l_a * WR) * input_ref[0,:] - WS / (2 * l_a * WR) * input_ref[1,:]
            # ur_pred = 1.0 / (r_a * WR) * upred[0,:] + WS / (2 * r_a * WR) * upred[1,:]
            # ul_pred = 1.0 / (l_a * WR) * upred[0,:] - WS / (2 * l_a * WR) * upred[1,:]

            # get next state
            xnext = ocp_solver.get(1, "x") + 0.001 * np.random.randn(3)
            # print(xnext)
            t_sim += Ts

            simT[0,i + 1] = t_sim
            for j in range(nx):
                simX[j,i + 1] = xnext[j]
        
        time.sleep(self.Ts)
        goal_handle.succeed()
        result.success = True
        return result
    
    def export_unicycle_ode_model_with_LocalConstraints(self):
        model_name = 'unicycle_ode'
        # set up states & controls
        x_pos       = SX.sym('x_pos')
        y_pos       = SX.sym('y_pos')
        theta_orient   = SX.sym('theta_orient')
        
        x = vertcat(x_pos, y_pos, theta_orient)

        v      = SX.sym('v')
        omega  = SX.sym('omega')
        
        u = vertcat(v , omega)

        # xdot
        x_pos_dot      = SX.sym('x_pos_dot')
        y_pos_dot      = SX.sym('y_pos_dot')
        theta_orient_dot   = SX.sym('theta_orient_dot')

        xdot = vertcat(x_pos_dot, y_pos_dot, theta_orient_dot)

        # parameters
        x_ref = SX.sym('x_ref') 
        y_ref = SX.sym('y_ref')
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
        ref_T = 20
        t_vec = t + np.linspace(t,t + N * Ts, N + 1)
        x_pos_ref = ref_rad * np.cos(2 * math.pi / ref_T * t_vec)
        y_pos_ref = ref_rad * np.sin(2 * math.pi / ref_T * t_vec)
        v_ref = 2 * math.pi * ref_rad / ref_T * np.ones(N)
        omega_ref = 2 * math.pi / ref_T * np.ones(N)
        theta_ref = math.pi / 2 + t_vec * 2 * math.pi / ref_T 
        state_ref = np.vstack((x_pos_ref.reshape(1,N + 1), y_pos_ref.reshape(1,N + 1), theta_ref.reshape(1,N + 1)))
        input_ref = np.vstack((v_ref.reshape(1,N), omega_ref.reshape(1,N)))
        return state_ref, input_ref

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
        executor.shutdown()
        node.destroy_node()
    return
    


if __name__ == '__main__':
    main()