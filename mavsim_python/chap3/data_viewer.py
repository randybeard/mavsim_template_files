from state_plotter.Plotter import Plotter
from state_plotter.plotter_args import *

class data_viewer:
    def __init__(self):
        self.plotter = Plotter(plotting_frequency=100)  # refresh plot every 100 time steps

        # set up the plot window
        # define first row
        pn_plots = PlotboxArgs(plots=['pn', 'pn_e'],
                               labels={'left': 'pn(m)', 'bottom': 'Time (s)'})
        pe_plots = PlotboxArgs(plots=['pe', 'pe_e'],
                               labels={'left': 'pe(m)', 'bottom': 'Time (s)'})
        h_plots = PlotboxArgs(plots=['h', 'h_e', 'h_c'],
                              labels={'left': 'h(m)', 'bottom': 'Time (s)'})
        wind_plots = PlotboxArgs(plots=['wn', 'wn_e', 'we', 'we_e'],
                                 labels={'left': 'wind(m/s)', 'bottom': 'Time (s)'})
        first_row = [pn_plots, pe_plots, h_plots, wind_plots]

        # define second row
        Va_plots = PlotboxArgs(plots=['Va', 'Va_e', 'Va_c'],
                               labels={'left': 'Va(m/s)', 'bottom': 'Time (s)'})
        alpha_plots = PlotboxArgs(plots=['alpha', 'alpha_e'],
                                  labels={'left': 'alpha(deg)', 'bottom': 'Time (s)'},
                                  rad2deg=True)
        beta_plots = PlotboxArgs(plots=['beta', 'beta_e'],
                                 labels={'left': 'beta(deg)', 'bottom': 'Time (s)'},
                                 rad2deg=True)
        Vg_plots = PlotboxArgs(plots=['Vg', 'Vg_e'],
                               labels={'left': 'Vg(m/s)', 'bottom': 'Time (s)'})
        second_row = [Va_plots, alpha_plots, beta_plots, Vg_plots]

        # define third row
        phi_plots = PlotboxArgs(plots=['phi', 'phi_e', 'phi_c'],
                                labels={'left': 'phi(deg)', 'bottom': 'Time (s)'},
                                rad2deg=True)
        theta_plots = PlotboxArgs(plots=['theta', 'theta_e', 'theta_c'],
                                  labels={'left': 'theta(deg)', 'bottom': 'Time (s)'},
                                  rad2deg=True)
        psi_plots = PlotboxArgs(plots=['psi', 'psi_e'],
                                labels={'left': 'psi(deg)', 'bottom': 'Time (s)'},
                                rad2deg=True)
        chi_plots = PlotboxArgs(plots=['chi', 'chi_e', 'chi_c'],
                                labels={'left': 'chi(deg)', 'bottom': 'Time (s)'},
                                rad2deg=True)
        third_row = [phi_plots, theta_plots, psi_plots, chi_plots]

        # define fourth row
        p_plots = PlotboxArgs(plots=['p', 'p_e'],
                              labels={'left': 'p(deg/s)', 'bottom': 'Time (s)'},
                              rad2deg=True)
        q_plots = PlotboxArgs(plots=['q', 'q_e'],
                              labels={'left': 'q(deg/s)', 'bottom': 'Time (s)'},
                              rad2deg=True)
        r_plots = PlotboxArgs(plots=['r', 'r_e'],
                              labels={'left': 'r(deg)', 'bottom': 'Time (s)'},
                              rad2deg=True)
        gyro_plots = PlotboxArgs(plots=['bx', 'bx_e', 'by', 'by_e', 'bz', 'bz_e'],
                                 labels={'left': 'bias(deg/s)', 'bottom': 'Time (s)'},
                                 rad2deg=True)
        fourth_row = [p_plots, q_plots, r_plots, gyro_plots]
        plots = [first_row,
                 second_row,
                 third_row,
                 fourth_row
                 ]

        # Add plots to the window
        self.plotter.add_plotboxes(plots)

        # Define and label vectors for more convenient/natural data input
        self.plotter.define_input_vector('true_state', ['pn', 'pe', 'h', 'Va', 'alpha', 'beta', 'phi', 'theta', 'chi',
                                                        'p', 'q', 'r', 'Vg', 'wn', 'we', 'psi', 'bx', 'by', 'bz'])
        self.plotter.define_input_vector('estimated_state', ['pn_e', 'pe_e', 'h_e', 'Va_e', 'alpha_e', 'beta_e',
                                                             'phi_e', 'theta_e', 'chi_e', 'p_e', 'q_e', 'r_e',
                                                             'Vg_e', 'wn_e', 'we_e', 'psi_e', 'bx_e', 'by_e', 'bz_e'])
        self.plotter.define_input_vector('commands', ['h_c', 'Va_c', 'phi_c', 'theta_c', 'chi_c'])

        # plot timer
        self.time = 0.

    def update(self, true_state, estimated_state, commanded_state, ts):
        commands = [commanded_state.h, # h_c
                    commanded_state.Va, # Va_c
                    commanded_state.phi, # phi_c
                    commanded_state.theta, # theta_c
                    commanded_state.chi] # chi_c
        ## Add the state data in vectors
        # the order has to match the order in lines 72-76
        true_state_list = [true_state.pn, true_state.pe, true_state.h,
                           true_state.Va, true_state.alpha, true_state.beta,
                           true_state.phi, true_state.theta, true_state.chi,
                           true_state.p, true_state.q, true_state.r,
                           true_state.Vg, true_state.wn, true_state.we, true_state.psi,
                           true_state.bx, true_state.by, true_state.bz]
        estimated_state_list = [estimated_state.pn, estimated_state.pe, estimated_state.h,
                                estimated_state.Va, estimated_state.alpha, estimated_state.beta,
                                estimated_state.phi, estimated_state.theta, estimated_state.chi,
                                estimated_state.p, estimated_state.q, estimated_state.r,
                                estimated_state.Vg, estimated_state.wn, estimated_state.we, estimated_state.psi,
                                estimated_state.bx, estimated_state.by, estimated_state.bz]
        self.plotter.add_vector_measurement('true_state', true_state_list, self.time)
        self.plotter.add_vector_measurement('estimated_state', estimated_state_list, self.time)
        self.plotter.add_vector_measurement('commands', commands, self.time)

        # Update and display the plot
        self.plotter.update_plots()

        # increment time
        self.time += ts



