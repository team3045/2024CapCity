// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commons;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * A version of {@link PS4Controller} with {@link GremlinTrigger} factories for command-based.
 *
 * @see PS4Controller
 */
@SuppressWarnings("MethodName")
public class GremlinPS4Controller extends GremlinCommandGenericHID {
  private final PS4Controller m_hid;

  /**
   * Construct an instance of a device.
   *
   * @param port The port index on the Driver Station that the device is plugged into.
   */
  public GremlinPS4Controller(int port) {
    super(port);
    m_hid = new PS4Controller(port);
  }

  /**
   * Get the underlying GenericHID object.
   *
   * @return the wrapped GenericHID object
   */
  @Override
  public PS4Controller getHID() {
    return m_hid;
  }

  /**
   * Constructs an event instance around the L2 button's digital signal.
   *
   * @return an event instance representing the L2 button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public GremlinTrigger L2() {
    return L2(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the L2 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the L2 button's digital signal attached to the given
   *     loop.
   */
  public GremlinTrigger L2(EventLoop loop) {
    return m_hid.L2(loop).castTo(GremlinTrigger::new);
  }

  /**
   * Constructs an event instance around the R2 button's digital signal.
   *
   * @return an event instance representing the R2 button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public GremlinTrigger R2() {
    return R2(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the R2 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the R2 button's digital signal attached to the given
   *     loop.
   */
  public GremlinTrigger R2(EventLoop loop) {
    return m_hid.R2(loop).castTo(GremlinTrigger::new);
  }

  /**
   * Constructs an event instance around the L1 button's digital signal.
   *
   * @return an event instance representing the L1 button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public GremlinTrigger L1() {
    return L1(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the L1 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the L1 button's digital signal attached to the given
   *     loop.
   */
  public GremlinTrigger L1(EventLoop loop) {
    return m_hid.L1(loop).castTo(GremlinTrigger::new);
  }

  /**
   * Constructs an event instance around the R1 button's digital signal.
   *
   * @return an event instance representing the R1 button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public GremlinTrigger R1() {
    return R1(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the R1 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the R1 button's digital signal attached to the given
   *     loop.
   */
  public GremlinTrigger R1(EventLoop loop) {
    return m_hid.R1(loop).castTo(GremlinTrigger::new);
  }

  /**
   * Constructs an event instance around the L3 button's digital signal.
   *
   * @return an event instance representing the L3 button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public GremlinTrigger L3() {
    return L3(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the L3 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the L3 button's digital signal attached to the given
   *     loop.
   */
  public GremlinTrigger L3(EventLoop loop) {
    return m_hid.L3(loop).castTo(GremlinTrigger::new);
  }

  /**
   * Constructs an event instance around the R3 button's digital signal.
   *
   * @return an event instance representing the R3 button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public GremlinTrigger R3() {
    return R3(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the R3 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the R3 button's digital signal attached to the given
   *     loop.
   */
  public GremlinTrigger R3(EventLoop loop) {
    return m_hid.R3(loop).castTo(GremlinTrigger::new);
  }

  /**
   * Constructs an event instance around the square button's digital signal.
   *
   * @return an event instance representing the square button's digital signal attached to the
   *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public GremlinTrigger square() {
    return square(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the square button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the square button's digital signal attached to the given
   *     loop.
   */
  public GremlinTrigger square(EventLoop loop) {
    return m_hid.square(loop).castTo(GremlinTrigger::new);
  }

  /**
   * Constructs an event instance around the cross button's digital signal.
   *
   * @return an event instance representing the cross button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public GremlinTrigger cross() {
    return cross(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the cross button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the cross button's digital signal attached to the given
   *     loop.
   */
  public GremlinTrigger cross(EventLoop loop) {
    return m_hid.cross(loop).castTo(GremlinTrigger::new);
  }

  /**
   * Constructs an event instance around the triangle button's digital signal.
   *
   * @return an event instance representing the triangle button's digital signal attached to the
   *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public GremlinTrigger triangle() {
    return triangle(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the triangle button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the triangle button's digital signal attached to the
   *     given loop.
   */
  public GremlinTrigger triangle(EventLoop loop) {
    return m_hid.triangle(loop).castTo(GremlinTrigger::new);
  }

  /**
   * Constructs an event instance around the circle button's digital signal.
   *
   * @return an event instance representing the circle button's digital signal attached to the
   *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public GremlinTrigger circle() {
    return circle(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the circle button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the circle button's digital signal attached to the given
   *     loop.
   */
  public GremlinTrigger circle(EventLoop loop) {
    return m_hid.circle(loop).castTo(GremlinTrigger::new);
  }

  /**
   * Constructs an event instance around the share button's digital signal.
   *
   * @return an event instance representing the share button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public GremlinTrigger share() {
    return share(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the share button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the share button's digital signal attached to the given
   *     loop.
   */
  public GremlinTrigger share(EventLoop loop) {
    return m_hid.share(loop).castTo(GremlinTrigger::new);
  }

  /**
   * Constructs an event instance around the PS button's digital signal.
   *
   * @return an event instance representing the PS button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public GremlinTrigger PS() {
    return PS(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the PS button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the PS button's digital signal attached to the given
   *     loop.
   */
  public GremlinTrigger PS(EventLoop loop) {
    return m_hid.PS(loop).castTo(GremlinTrigger::new);
  }

  /**
   * Constructs an event instance around the options button's digital signal.
   *
   * @return an event instance representing the options button's digital signal attached to the
   *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public GremlinTrigger options() {
    return options(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the options button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the options button's digital signal attached to the
   *     given loop.
   */
  public GremlinTrigger options(EventLoop loop) {
    return m_hid.options(loop).castTo(GremlinTrigger::new);
  }

  /**
   * Constructs an event instance around the touchpad's digital signal.
   *
   * @return an event instance representing the touchpad's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public GremlinTrigger touchpad() {
    return touchpad(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the touchpad's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the touchpad's digital signal attached to the given
   *     loop.
   */
  public GremlinTrigger touchpad(EventLoop loop) {
    return m_hid.touchpad(loop).castTo(GremlinTrigger::new);
  }

  /**
   * Get the X axis value of left side of the controller.
   *
   * @return the axis value.
   */
  public double getLeftX() {
    return m_hid.getLeftX();
  }

  /**
   * Get the X axis value of left side of the controller times negative one.
   *
   * @return the axis value times negative one.
   */
  public double getLeftXReversed() {
    return m_hid.getLeftX() * -1;
  }

  /**
   * Get the X axis value of right side of the controller.
   *
   * @return the axis value.
   */
  public double getRightX() {
    return m_hid.getRightX();
  }

  /**
   * Get the X axis value of right side of the controller times negative one.
   *
   * @return the axis value times negative one.
   */
  public double getRightXReversed() {
    return m_hid.getRightX() * -1;
  }

  /**
   * Get the Y axis value of left side of the controller.
   *
   * @return the axis value.
   */
  public double getLeftY() {
    return m_hid.getLeftY();
  }

  /**
   * Get the Y axis value of left side of the controller times negative one.
   *
   * @return the axis value times negative one.
   */
  public double getLeftYReversed(){
    return m_hid.getLeftY() * -1;
  }

  /**
   * Get the Y axis value of right side of the controller.
   *
   * @return the axis value.
   */
  public double getRightY() {
    return m_hid.getRightY();
  }

  /**
   * Get the L2 axis value of the controller. Note that this axis is bound to the range of [0, 1] as
   * opposed to the usual [-1, 1].
   *
   * @return the axis value.
   */
  public double getL2Axis() {
    return m_hid.getL2Axis();
  }

  /**
   * Get the R2 axis value of the controller. Note that this axis is bound to the range of [0, 1] as
   * opposed to the usual [-1, 1].
   *
   * @return the axis value.
   */
  public double getR2Axis() {
    return m_hid.getR2Axis();
  }
}
