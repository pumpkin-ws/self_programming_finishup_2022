package com.spark;
import redis.clients.jedis.JedisPool;
import redis.clients.jedis.JedisPoolConfig;

public class RedisClient {
    public static String IP = "127.0.0.1";
    public static int PORT = 6379;
    public static JedisPool jedisPool= new JedisPool(new JedisPoolConfig(), IP, PORT);
    Publisher
}
