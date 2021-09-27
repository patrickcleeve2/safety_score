#!/usr/bin/env python3


import numpy as np
import pandas as pd
import streamlit as st

from safety_score_node import (LATERAL_ACCEL_BASELINE, LATERAL_ACCEL_THRESHOLD,
                               LONGITUDINAL_ACCEL_BASELINE,
                               LONGITUDINAL_ACCEL_THRESHOLD,
                               UNSAFE_FOLLOWING_BASELINE,
                               UNSAFE_FOLLOWING_THRESHOLD)

st.set_page_config(layout="wide")

df = pd.read_csv("data/data.csv")


df["HARD_BRAKING"] = df["field.linear_acceleration_x"] < -LONGITUDINAL_ACCEL_THRESHOLD
df["HARD_BRAKING_BASELINE"] = df["field.linear_acceleration_x"] < -LONGITUDINAL_ACCEL_BASELINE
df["AGGRESSIVE_TURNING"] = df["field.linear_acceleration_y"] > LATERAL_ACCEL_THRESHOLD
df["AGGRESSIVE_TURNING_BASELINE"] = df["field.linear_acceleration_y"] > LATERAL_ACCEL_BASELINE
df["UNSAFE_FOLLOWING"] = df["field.time_to_collision"] < UNSAFE_FOLLOWING_THRESHOLD
df["UNSAFE_FOLLOWING_BASELINE"] = df["field.time_to_collision"] < UNSAFE_FOLLOWING_BASELINE

df["field.forward_collision_warning"] = df["field.forward_collision_warning"].astype(bool) 

st.write(df)



st.markdown("---")
df_hard_brake = df.groupby(by=["HARD_BRAKING_BASELINE", "HARD_BRAKING"]).agg(
    count=pd.NamedAgg(column="field.header.stamp", aggfunc="count")
)
df_hard_brake.index.names = ["baseline", "threshold"]

st.write("HARD_BRAKING", df_hard_brake)


df_agg_turn = df.groupby(by=["AGGRESSIVE_TURNING_BASELINE", "AGGRESSIVE_TURNING"]).agg(
    count=pd.NamedAgg(column="field.header.stamp", aggfunc="count")
)

st.write("AGGRESIVE_TURNING", df_agg_turn)


df_unsafe_follow = df.groupby(by=["UNSAFE_FOLLOWING_BASELINE", "UNSAFE_FOLLOWING"]).agg(
    count=pd.NamedAgg(column="field.header.stamp", aggfunc="count")
)

st.write("UNSAFE_FOLLOWING", df_unsafe_follow)

df_fcw = df.groupby(by=["field.forward_collision_warning"]).agg(
    count=pd.NamedAgg(column="field.header.stamp", aggfunc="count")
)

st.write("FORWARD_COLLISION_WARNING", df_fcw)
